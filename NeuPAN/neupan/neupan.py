'''
neupan file is the main class for the NeuPan algorithm. It wraps the PAN class and provides a more user-friendly interface.

Developed by Ruihua Han
Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>

Modified by Haowen Yu.
Copyright (c) 2026 Manifold Tech Ltd <yuhaowen@manifoldtech.cn>

NeuPAN planner is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

NeuPAN planner is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with NeuPAN planner. If not, see <https://www.gnu.org/licenses/>.
'''

from __future__ import annotations

import yaml
import torch
import threading
from neupan.robot import robot
from neupan.blocks import InitialPath, PAN
from neupan import configuration
from neupan.util import time_it, file_check, get_transform
import numpy as np
from neupan.configuration import np_to_tensor, tensor_to_np
from math import cos, sin

class neupan(torch.nn.Module):

    """
    Args:
        receding: int, the number of steps in the receding horizon.
        step_time: float, the time step in the MPC framework.
        ref_speed: float, the reference speed of the robot.
        device: str, the device to run the algorithm on. 'cpu' or 'cuda'.
        robot_kwargs: dict, the keyword arguments for the robot class.
        ipath_kwargs: dict, the keyword arguments for the initial path class.
        pan_kwargs: dict, the keyword arguments for the PAN class.
        adjust_kwargs: dict, the keyword arguments for the adjust class
        train_kwargs: dict, the keyword arguments for the train class
        time_print: bool, whether to print the forward time of the algorithm.
        collision_threshold: float, the threshold for the collision detection. If collision, the algorithm will stop.
    """

    def __init__(
        self,
        receding: int = 10,
        step_time: float = 0.1,
        ref_speed: float = 4.0,
        device: str = "cpu",
        robot_kwargs: dict = None,
        ipath_kwargs: dict = None,
        pan_kwargs: dict = None,
        adjust_kwargs: dict = None,
        train_kwargs: dict = None,
        **kwargs,
    ) -> None:
        super(neupan, self).__init__()

        # mpc parameters
        self.T = receding
        self.dt = step_time
        self.ref_speed = ref_speed

        configuration.device = torch.device(device)
        configuration.time_print = kwargs.get("time_print", False)
        self.collision_threshold = kwargs.get("collision_threshold", 0.1)

        # initialization
        self.cur_vel_array = np.zeros((2, self.T))
        self.robot = robot(receding, step_time, **robot_kwargs)

        self.ipath = InitialPath(
            receding, step_time, ref_speed, self.robot, **ipath_kwargs
        )
        self._path_lock = threading.RLock()
        self._path_generation = 0
            
        pan_kwargs["adjust_kwargs"] = adjust_kwargs
        pan_kwargs["train_kwargs"] = train_kwargs
        self.dune_train_kwargs = train_kwargs

        self.pan = PAN(receding, step_time, self.robot, **pan_kwargs)

        self.info = {"stop": False, "arrive": False, "collision": False}

    @classmethod
    def init_from_yaml(cls, yaml_file, **kwargs):
        abs_path = file_check(yaml_file)

        with open(abs_path, "r") as f:
            config = yaml.safe_load(f)
            for key, value in kwargs.items():
                if isinstance(config.get(key), dict) and isinstance(value, dict):
                    config[key].update(value)
                else:
                    config[key] = value

        config["robot_kwargs"] = config.pop("robot", dict())
        config["ipath_kwargs"] = config.pop("ipath", dict())
        config["pan_kwargs"] = config.pop("pan", dict())
        config["adjust_kwargs"] = config.pop("adjust", dict())
        config["train_kwargs"] = config.pop("train", dict())

        return cls(**config)

    @time_it("neupan forward")
    def forward(self, state, points, velocities=None):
        """
        state: current state of the robot, matrix (3, 1), x, y, theta
        points: current input obstacle point positions, matrix (2, N), N is the number of obstacle points.
        velocities: current velocity of each obstacle point, matrix (2, N), N is the number of obstacle points. vx, vy
        """

        assert state.shape[0] >= 3

        # Path rebuilding clears curve_list temporarily. Serialize only the
        # short path/reference operations, never the expensive optimizer.
        with self._path_lock:
            info = self.info.copy()
            info["arrive"] = bool(self.ipath.check_arrive(state))
            if info["arrive"]:
                self.info = info
                return np.zeros((2, 1)), info
            nom_input_np = self.ipath.generate_nom_ref_state(
                state, self.cur_vel_array, self.ref_speed
            )
            path_generation = self._path_generation

        # convert to tensor
        nom_input_tensor = [np_to_tensor(n) for n in nom_input_np]
        obstacle_points_tensor = np_to_tensor(points) if points is not None else None
        point_velocities_tensor = (
            np_to_tensor(velocities) if velocities is not None else None
        )

        opt_state_tensor, opt_vel_tensor, opt_distance_tensor = self.pan(
            *nom_input_tensor, obstacle_points_tensor, point_velocities_tensor
        )

        opt_state_np, opt_vel_np = tensor_to_np(opt_state_tensor), tensor_to_np(
            opt_vel_tensor
        )

        info["state_tensor"] = opt_state_tensor
        info["vel_tensor"] = opt_vel_tensor
        info["distance_tensor"] = opt_distance_tensor
        info['ref_state_tensor'] = nom_input_tensor[2]
        info['ref_speed_tensor'] = nom_input_tensor[3]

        info["ref_state_list"] = [
            state[:, np.newaxis] for state in nom_input_np[2].T
        ]
        info["opt_state_list"] = [state[:, np.newaxis] for state in opt_state_np.T]
        info["stop"] = bool(self.check_stop())

        with self._path_lock:
            # A stop or replacement can complete while PAN is solving. Its old
            # result must not overwrite the new path's reset/warm-start state.
            if path_generation == self._path_generation:
                self.cur_vel_array = opt_vel_np
                self.info = info

        action = np.zeros((2, 1)) if info["stop"] else opt_vel_np[:, 0:1]
        return action, info

    def check_stop(self):
        return self.min_distance < self.collision_threshold
    

    def scan_to_point(
        self,
        state: np.ndarray,
        scan: dict,
        scan_offset: list[float] = [0, 0, 0],
        angle_range: list[float] = [-np.pi, np.pi],
        down_sample: int = 1,
    ) -> np.ndarray | None:
        
        """
        input:
            state: [x, y, theta]
            scan: {}
                ranges: list[float], the range of the scan
                angle_min: float, the minimum angle of the scan
                angle_max: float, the maximum angle of the scan
                range_max: float, the maximum range of the scan
                range_min: float, the minimum range of the scan

            scan_offset: [x, y, theta], the relative position of the sensor to the robot state coordinate

        return point cloud: (2, n)
        """
        # Vectorized scan-to-point conversion
        ranges = np.asarray(scan["ranges"], dtype=float)
        if ranges.size == 0:
            return None

        angles = np.linspace(scan["angle_min"], scan["angle_max"], len(ranges))

        valid = (
            (ranges < (scan["range_max"] - 0.02))
            & (ranges > scan["range_min"])
            & (angles > angle_range[0])
            & (angles < angle_range[1])
        )

        if not np.any(valid):
            return None

        r = ranges[valid]
        a = angles[valid]
        point_array = np.vstack((r * np.cos(a), r * np.sin(a)))

        s_trans, s_R = get_transform(np.c_[scan_offset])
        temp_points = s_R @ point_array + s_trans

        trans, R = get_transform(state)
        points = (R @ temp_points + trans)[:, ::down_sample]

        return points

    def scan_to_point_velocity(
        self,
        state,
        scan,
        scan_offset=[0, 0, 0],
        angle_range=[-np.pi, np.pi],
        down_sample=1,
    ):
        """
        input:
            state: [x, y, theta]
            scan: {}
                ranges: list[float], the ranges of the scan
                angle_min: float, the minimum angle of the scan
                angle_max: float, the maximum angle of the scan
                range_max: float, the maximum range of the scan
                range_min: float, the minimum range of the scan
                velocity: list[float], the velocity of the scan

            scan_offset: [x, y, theta], the relative position of the sensor to the robot state coordinate

        return point cloud: (2, n)
        """
        point_cloud = []
        velocity_points = []

        ranges = np.array(scan["ranges"])
        angles = np.linspace(scan["angle_min"], scan["angle_max"], len(ranges))
        scan_velocity = scan.get("velocity", np.zeros((2, len(ranges))))

        # lidar_state = self.lidar_state_transform(state, np.c_[self.lidar_offset])
        for i in range(len(ranges)):
            scan_range = ranges[i]
            angle = angles[i]

            if scan_range < (scan["range_max"] - 0.02) and scan_range >= scan["range_min"]:
                if angle > angle_range[0] and angle < angle_range[1]:
                    point = np.array(
                        [[scan_range * cos(angle)], [scan_range * sin(angle)]]
                    )
                    point_cloud.append(point)
                    velocity_points.append(scan_velocity[:, i : i + 1])

        if len(point_cloud) == 0:
            return None, None

        point_array = np.hstack(point_cloud)
        s_trans, s_R = get_transform(np.c_[scan_offset])
        temp_points = s_R.T @ (
            point_array - s_trans
        )  # points in the robot state coordinate

        trans, R = get_transform(state)
        points = (R @ temp_points + trans)[:, ::down_sample]

        velocity = np.hstack(velocity_points)[:, ::down_sample]

        return points, velocity


    def train_dune(self):
        self.pan.dune_layer.train_dune(self.dune_train_kwargs)


    def reset(self, reset_velocity: bool = False, clear_path: bool = False):
        """
        Reset path-tracking state.

        By default the MPC nominal velocity (`cur_vel_array`) is
        preserved across calls. Earlier this method always zeroed it, which
        caused the planner to forget its momentum every time a goal-state
        machine re-published the same path (~5 Hz), producing ``cmd_vel`` of
        zero forever. Pass ``reset_velocity=True`` only when a true cold
        start is desired (e.g. arrived + new task). ``clear_path`` invalidates
        the active reference while retaining waypoints for existing callers.
        """

        with self._path_lock:
            self._path_generation += 1
            if clear_path:
                self.ipath.initial_path = None
            self.ipath.point_index = 0
            self.ipath.curve_index = 0
            self.ipath.arrive_flag = False
            self.info["stop"] = False
            self.info["arrive"] = False
            if reset_velocity:
                self.cur_vel_array = np.zeros_like(self.cur_vel_array)

    def set_initial_path(self, path):

        '''
        set the initial path from the given path
        path: list of [x, y, theta, gear] 4x1 vector, gear is -1 (back gear) or 1 (forward gear)
        '''

        with self._path_lock:
            self._path_generation += 1
            self.ipath.set_initial_path(path)

    def set_initial_path_from_state(self, state):
        """
        Args:
            states: [x, y, theta] or 3x1 vector

        """
        with self._path_lock:
            self._path_generation += 1
            self.ipath.init_check(state)
    
    def set_reference_speed(self, speed: float):

        """
        Args:
            speed: float, the reference speed of the robot
        """

        self.ipath.ref_speed = speed
        self.ref_speed = speed
    
    def update_initial_path_from_goal(self, start, goal):

        """
        Args:
            start: [x, y, theta] or 3x1 vector
            goal: [x, y, theta] or 3x1 vector
        """

        with self._path_lock:
            self._path_generation += 1
            self.ipath.update_initial_path_from_goal(start, goal)


    def update_initial_path_from_waypoints(self, waypoints):

        """
        Args:
            waypoints: list of [x, y, theta] or 3x1 vector
        """

        with self._path_lock:
            self._path_generation += 1
            self.ipath.set_ipath_with_waypoints(waypoints)


    def update_adjust_parameters(self, **kwargs):

        """
        Update the adjust parameter values online (no node restart needed).

        Args:
            q_s: weight of state cost (xy tracking)
            q_theta: weight of state cost (theta tracking, B1)
            p_u: weight of linear-velocity tracking cost
            p_w: weight of angular-velocity regularizer (omega -> 0, B2)
            p_dw: weight of angular-velocity RATE regularizer (omega[t+1]-omega[t]
                  -> 0). Use this to suppress oscillation around zero that
                  p_w alone cannot dampen.
            eta: weight of collision-avoidance distance reward
            d_max: maximum safety distance
            d_min: minimum safety distance

        NOTE: ro_obs and bk are NOT online-tunable (DPP-incompatible if
        promoted to cvxpy Parameters). Set them in config and restart.
        """

        self.pan.nrmp_layer.update_adjust_parameters_value(**kwargs)

    @property
    def min_distance(self):
        return self.pan.min_distance
    
    @property
    def dune_points(self):
        return self.pan.dune_points
    
    @property
    def nrmp_points(self):
        return self.pan.nrmp_points
    
    @property
    def initial_path(self):
        return self.ipath.initial_path

    @initial_path.setter
    def initial_path(self, value):
        """
        Explicit setter so callers can do
        ``planner.initial_path = None`` without tripping PyTorch 2.4+'s
        stricter ``nn.Module.__setattr__`` (which rejects assignment to a
        read-only ``@property``).
        """
        self.ipath.initial_path = value

    @property
    def adjust_parameters(self):
        return self.pan.nrmp_layer.adjust_parameters
    
    @property
    def waypoints(self):

        '''
        Waypoints for generating the initial path
        '''

        return self.ipath.waypoints
    
    @property
    def opt_trajectory(self):

        '''
        MPC receding horizon trajectory under the velocity sequence
        return a list of state sequence, each state is a 3x1 vector
        '''

        return self.info["opt_state_list"]
    
    @property
    def ref_trajectory(self):

        '''
        Reference trajectory on the initial path
        return a list of state sequence, each state is a 3x1 vector
        '''

        return self.info["ref_state_list"]

    

    
