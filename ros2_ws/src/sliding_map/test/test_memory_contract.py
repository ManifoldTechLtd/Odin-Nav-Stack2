#!/usr/bin/python3
# Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#   http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit and regression tests for the sliding_map map memory contract."""
import math
import os
import subprocess
import threading
import time
import uuid

import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

RES, NW, STEP, RMAXF = 0.05, 400, 0.025, 4.5
FP_L, FP_W, FP_M = 0.96, 0.55, 0.10
ODIN_TO_BASE = np.array([-0.42, 0.0, 0.12])
POSE = np.eye(4)
POINT = np.array([1.25, 0.75])
ANGLE = float(np.arctan2(POINT[1], POINT[0]))
RANGE = float(np.linalg.norm(POINT))

VARIANTS = {
    'base15': dict(static=15.0, dyn=15.0, thr=None, sticky=False, require_vacated=False),
    'glob2': dict(static=2.0, dyn=2.0, thr=None, sticky=False, require_vacated=False),
    'dynNS05': dict(static=15.0, dyn=2.0, thr=0.05, sticky=False, require_vacated=False),
    'dynNS10': dict(static=15.0, dyn=2.0, thr=0.10, sticky=False, require_vacated=False),
    'dynST10': dict(static=15.0, dyn=2.0, thr=0.10, sticky=True, require_vacated=False),
    'dynVacated': dict(static=15.0, dyn=2.0, thr=0.10, sticky=False, require_vacated=True),
}


class EmptyStaticMap:
    def near_occ(self, x, y):
        return np.zeros(len(x), dtype=bool)


def dynamic_fraction(pb, po, prev_po):
    """Compute per-cluster unsupported point fraction (>6 cm in odom frame relative to previous frame)."""
    frac = np.zeros(len(pb))
    rh = np.hypot(pb[:, 0], pb[:, 1])
    idx = np.flatnonzero((pb[:, 0] > -0.2) & (rh < 4.0) & (pb[:, 2] > -0.40) & (pb[:, 2] < 1.5))
    if prev_po is None or len(idx) == 0:
        return frac
    uns = cKDTree(prev_po).query(po[idx], distance_upper_bound=0.5)[0] > 0.06
    vk = np.floor(pb[idx] / 0.05).astype(np.int64)
    uniq, inv = np.unique(vk, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    n = len(uniq)
    pairs = cKDTree(uniq.astype(float)).query_pairs(1.75, output_type='ndarray')
    _, comp = connected_components(
        coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(n, n)),
        directed=False)
    k = comp[inv]
    cu = np.bincount(k, weights=uns.astype(float), minlength=comp.max() + 1)
    cn = np.bincount(k, minlength=comp.max() + 1)
    frac[idx] = (cu / np.maximum(cn, 1))[k]
    return frac


class SlidingMap:
    def __init__(self, cfg, origin):
        self.cfg = cfg
        self.ox, self.oy = origin
        n = NW * NW
        self.grid = np.full(n, -1, np.int8)
        self.stamp = np.zeros(n)
        self.dyn = np.zeros(n, bool)
        self.vacated = np.zeros(n, bool)
        self.seen_free = np.zeros(n, bool)
        self.last_check = None
        self.st = {}

    def _shift(self, sx, sy):
        for name in ('grid', 'stamp', 'dyn', 'vacated', 'seen_free'):
            a = getattr(self, name).reshape(NW, NW)
            fill = {'grid': -1, 'stamp': 0}.get(name, False)
            b = np.full_like(a, fill)
            ys, yd = (slice(sy, NW), slice(0, NW - sy)) if sy >= 0 else (slice(0, NW + sy), slice(-sy, NW))
            xs, xd = (slice(sx, NW), slice(0, NW - sx)) if sx >= 0 else (slice(0, NW + sx), slice(-sx, NW))
            if abs(sx) < NW and abs(sy) < NW:
                b[yd, xd] = a[ys, xs]
            setattr(self, name, b.reshape(-1))
        self.ox += sx * RES
        self.oy += sy * RES

    def recenter(self, x, y):
        half = NW * RES / 2
        if abs(x - (self.ox + half)) < 5.0 and abs(y - (self.oy + half)) < 5.0:
            return
        sx = int(round((x - half - self.ox) / RES))
        sy = int(round((y - half - self.oy) / RES))
        self._shift(sx, sy)

    def lin(self, x, y):
        c = np.floor((x - self.ox) / RES).astype(np.int64)
        r = np.floor((y - self.oy) / RES).astype(np.int64)
        ok = (c >= 0) & (c < NW) & (r >= 0) & (r < NW)
        return r * NW + c, ok

    def step(self, T, ranges, amin, ainc, rmin, rmax, now, bdyn, smap, T_map_odom, counting):
        o, R = T[:3, 3], T[:3, :3]
        self.recenter(o[0], o[1])
        Tb = T.copy()
        Tb[:3, 3] = o + R @ ODIN_TO_BASE
        inv_b = np.linalg.inv(Tb)
        nb = len(ranges)
        ang = amin + ainc * np.arange(nb)
        valid_beam = ~np.isnan(ranges)
        fin = np.isfinite(ranges)
        rng = np.where(fin, ranges, rmax)
        hit = fin & (ranges >= rmin) & (ranges < rmax)
        flim = np.where(hit, np.maximum(rng - 1e-3, 0), np.where(valid_beam, np.minimum(rmax, rng), 0.0))
        nstep = np.floor(flim / STEP + 1e-9).astype(int)
        bidx = np.repeat(np.arange(nb), nstep)
        dist = (np.arange(nstep.sum()) - np.repeat(np.cumsum(nstep) - nstep, nstep) + 1) * STEP
        dx = np.cos(ang[bidx]) * dist
        dy = np.sin(ang[bidx]) * dist
        px = o[0] + R[0, 0] * dx + R[0, 1] * dy
        py = o[1] + R[1, 0] * dx + R[1, 1] * dy
        valid_origins = np.flatnonzero(valid_beam)
        fidx, fok = self.lin(np.r_[ np.full(len(valid_origins), o[0]), px],
                             np.r_[np.full(len(valid_origins), o[1]), py])
        fbeam = np.r_[valid_origins, bidx]
        free_max = np.full(NW * NW, -1)
        np.maximum.at(free_max, fidx[fok], fbeam[fok])
        hb = np.flatnonzero(hit & (ranges <= RMAXF))
        hx = o[0] + R[0, 0] * np.cos(ang[hb]) * ranges[hb] + R[0, 1] * np.sin(ang[hb]) * ranges[hb]
        hy = o[1] + R[1, 0] * np.cos(ang[hb]) * ranges[hb] + R[1, 1] * np.sin(ang[hb]) * ranges[hb]
        bx = inv_b[0, 0] * hx + inv_b[0, 1] * hy + inv_b[0, 3]
        by = inv_b[1, 0] * hx + inv_b[1, 1] * hy + inv_b[1, 3]
        keep = ~((np.abs(bx) <= FP_L / 2 + FP_M) & (np.abs(by) <= FP_W / 2 + FP_M))
        hidx, hok = self.lin(hx[keep], hy[keep])
        hit_max = np.full(NW * NW, -1)
        np.maximum.at(hit_max, hidx[hok], hb[keep][hok])
        occ_f = (hit_max >= 0) & (hit_max >= free_max)
        free_f = (free_max >= 0) & ~occ_f
        was_unknown = self.grid == -1
        self.grid[free_f] = 0
        self.dyn[free_f] = False
        self.vacated[free_f] = False
        self.seen_free[free_f] = True
        self.grid[occ_f] = 100
        self.stamp[occ_f | free_f] = now
        thr = self.cfg['thr']
        if thr is not None:
            d = bdyn[hit_max[occ_f]] >= thr
            if self.cfg.get('require_vacated', False):
                d = d & ~(was_unknown[occ_f] & ~self.seen_free[occ_f])
            self.dyn[occ_f] = (self.dyn[occ_f] | d) if self.cfg['sticky'] else d
            self.vacated[occ_f] = False
        if self.cfg.get('require_vacated', False):
            active_dyn = np.flatnonzero((self.grid == 100) & self.dyn & ~occ_f)
            dyn_hits = np.flatnonzero(occ_f & self.dyn)
            if len(active_dyn) and len(dyn_hits):
                cx = self.ox + (active_dyn % NW + 0.5) * RES
                cy = self.oy + (active_dyn // NW + 0.5) * RES
                dx = self.ox + (dyn_hits % NW + 0.5) * RES
                dy = self.oy + (dyn_hits // NW + 0.5) * RES
                dists = np.hypot(cx[:, None] - dx[None, :], cy[:, None] - dy[None, :])
                moved = ((dists >= 0.12) & (dists <= 0.85)).any(axis=1)
                self.vacated[active_dyn[moved]] = True
        if self.last_check is None or now < self.last_check:
            self.last_check = now
        elif now - self.last_check >= 0.5:
            self.last_check = now
            if self.cfg.get('require_vacated', False):
                to = np.where(self.vacated, self.cfg['dyn'], self.cfg['static'])
            else:
                to = np.where(self.dyn, self.cfg['dyn'], self.cfg['static'])
            drop = (self.grid == 100) & (self.stamp > 0) & (now - self.stamp > to)
            if drop.any():
                ii = np.flatnonzero(drop)
                self.grid[ii] = -1
                self.stamp[ii] = 0
                self.dyn[ii] = False
                self.vacated[ii] = False


def make_sim(variant):
    sim = SlidingMap(VARIANTS[variant], (-10.0, -10.0))
    idx, ok = sim.lin(POINT[:1], POINT[1:])
    assert ok.all()
    return sim, int(idx[0])


def observe(sim, stamp, hit=True, dynamic=0.0, point=POINT, pose=POSE):
    ang = float(np.arctan2(point[1] - pose[1, 3], point[0] - pose[0, 3]))
    rng = float(np.hypot(point[0] - pose[0, 3], point[1] - pose[1, 3]))
    ranges = np.array([rng if hit else np.inf])
    sim.step(pose, ranges, ang, 0.017, 0.2, 5.0, stamp,
             np.array([dynamic]), EmptyStaticMap(), pose, True)


def blind(sim, stamp, pose=POSE):
    sim.step(pose, np.array([]), ANGLE, 0.017, 0.2, 5.0, stamp,
             np.array([]), EmptyStaticMap(), pose, True)


def test_visible_obstacle_is_renewed_beyond_15_seconds():
    sim, idx = make_sim('base15')
    for stamp in np.arange(10.0, 40.1, 0.1):
        observe(sim, float(stamp))
        assert sim.grid[idx] == 100


def test_blind_static_retains_15_seconds_since_last_hit():
    sim, idx = make_sim('base15')
    observe(sim, 10.0)
    blind(sim, 24.9)
    assert sim.grid[idx] == 100
    blind(sim, 25.6)
    assert sim.grid[idx] == -1


def test_confirmed_free_clears_before_timeout():
    sim, idx = make_sim('base15')
    observe(sim, 10.0)
    observe(sim, 10.1, hit=False)
    assert sim.grid[idx] == 0


def test_new_stationary_box_also_has_high_two_frame_difference():
    yz = np.stack(np.meshgrid(np.linspace(0.3, 0.7, 20),
                              np.linspace(-0.3, 1.0, 30)), axis=-1).reshape(-1, 2)
    box = np.column_stack((np.full(len(yz), 1.25), yz))
    far_scene = box.copy()
    far_scene[:, 0] = 2.5
    first = dynamic_fraction(box, box, far_scene)
    repeated = dynamic_fraction(box, box, box)
    assert (first >= 0.10).all()
    assert (repeated == 0.0).all()


def test_static_misclassified_once_is_forgotten_after_2_seconds_without_vacated_guard():
    sim, idx = make_sim('dynNS05')
    observe(sim, 10.0, dynamic=1.0)
    blind(sim, 12.6)
    assert sim.grid[idx] == -1
    sim_v, idx_v = make_sim('dynVacated')
    observe(sim_v, 10.0, dynamic=1.0)
    blind(sim_v, 24.9)
    assert sim_v.grid[idx_v] == 100


def test_sticky_motion_label_survives_even_after_target_stops():
    sim, idx = make_sim('dynST10')
    observe(sim, 10.0, dynamic=1.0)
    for stamp in np.arange(10.1, 13.1, 0.1):
        observe(sim, float(stamp), dynamic=0.0)
    assert sim.dyn[idx]
    blind(sim, 15.6)
    assert sim.grid[idx] == -1
    sim, idx = make_sim('dynVacated')
    observe(sim, 10.0, dynamic=1.0)
    observe(sim, 13.0, dynamic=0.0)
    blind(sim, 15.6)
    assert sim.grid[idx] == 100


def test_t1_to_t7_truth_regressions():
    """T1-T7 truth regression scenarios."""
    # T1: Single-frame appearance before entering blind spot retains occupancy >= 15s
    sim, idx = make_sim('dynVacated')
    observe(sim, 10.0, dynamic=1.0)
    blind(sim, 24.9)
    assert sim.grid[idx] == 100

    # T2: Slow detour leaving FOV for 2-15s retains occupancy
    sim, idx = make_sim('dynVacated')
    observe(sim, 10.0, dynamic=1.0)
    observe(sim, 10.1, dynamic=0.0)
    for t in np.arange(10.5, 22.5, 0.5):
        blind(sim, float(t))
        assert sim.grid[idx] == 100

    # T3: Moving pedestrian stops and enters blind spot -> retains occupancy >= 15s
    sim, idx = make_sim('dynVacated')
    observe(sim, 9.8, hit=False)
    observe(sim, 10.0, dynamic=1.0)
    observe(sim, 10.2, dynamic=0.0)
    blind(sim, 25.0)
    assert sim.grid[idx] == 100

    # T4: Pedestrian moves across cells: vacated cell expires quickly while current position is retained
    sim, idx_old = make_sim('dynVacated')
    pt_new = np.array([1.275, 0.475])
    idx_new, _ = sim.lin(pt_new[:1], pt_new[1:])
    idx_new = int(idx_new[0])
    observe(sim, 9.8, hit=False, point=POINT)
    observe(sim, 10.0, dynamic=1.0, point=POINT)
    observe(sim, 10.1, hit=False, point=pt_new)
    observe(sim, 10.2, dynamic=1.0, point=pt_new)
    for t in np.arange(10.5, 13.0, 0.5):
        blind(sim, float(t))
    assert sim.grid[idx_old] == -1, 'Vacated dynamic footprint should expire after short timeout'
    assert sim.grid[idx_new] == 100, 'Current dynamic target position must be retained'

    # T5: Occluded target behind a 0.70m obstacle retains occupancy for 15s
    sim, idx = make_sim('dynVacated')
    observe(sim, 10.0, dynamic=0.0, point=POINT)
    occluder = POINT * (0.70 / RANGE)
    for t in np.arange(10.5, 24.9, 0.5):
        observe(sim, float(t), dynamic=0.0, point=occluder)
    assert sim.grid[idx] == 100

    # T6: NaN scan frame does not clear existing occupied cells
    sim, idx = make_sim('dynVacated')
    observe(sim, 10.0, dynamic=0.0, point=POINT)
    sim.step(POSE, np.array([np.nan]), ANGLE, 0.017, 0.2, 5.0, 11.0,
             np.array([0.0]), EmptyStaticMap(), POSE, True)
    assert sim.grid[idx] == 100

    # T7: Hits inside robot footprint are ignored and recentering preserves cell state
    sim, idx = make_sim('dynVacated')
    observe(sim, 9.8, hit=False, point=POINT)
    observe(sim, 10.0, dynamic=1.0, point=POINT)
    near_fp = np.array([-0.10, 0.05])
    idx_fp, _ = sim.lin(near_fp[:1], near_fp[1:])
    observe(sim, 10.1, dynamic=0.0, point=near_fp)
    assert sim.grid[int(idx_fp[0])] != 100
    moved_pose = POSE.copy()
    moved_pose[0, 3] = 6.5
    blind(sim, 10.2, pose=moved_pose)
    idx_shifted, ok = sim.lin(POINT[:1], POINT[1:])
    assert ok.all()
    assert sim.grid[int(idx_shifted[0])] == 100
    assert sim.dyn[int(idx_shifted[0])]


def test_real_sliding_map_cpp_node_dynamic_memory_and_recenter(monkeypatch):
    """Integration test against the compiled sliding_map_node binary."""
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    domain = str(190 + os.getpid() % 15)
    monkeypatch.setenv('ROS_DOMAIN_ID', domain)
    binary = os.path.join(get_package_prefix('sliding_map'), 'lib', 'sliding_map', 'sliding_map_node')
    assert os.path.isfile(binary), f'sliding_map_node binary not found: {binary}'

    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=int(domain))
    node = rclpy.create_node('sliding_map_test_rig_' + uuid.uuid4().hex[:6], context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    maps = []
    qos_map = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/sliding_map', maps.append, qos_map)
    scan_pub = node.create_publisher(LaserScan, '/scan', 5)
    dyn_pub = node.create_publisher(LaserScan, '/scan_dynamic', 5)
    static_tf = StaticTransformBroadcaster(node)
    dyn_tf = TransformBroadcaster(node)

    stf = TransformStamped()
    stf.header.frame_id = 'odin1_base_link'
    stf.child_frame_id = 'base_link'
    stf.transform.translation.x = -0.42
    stf.transform.translation.z = 0.12
    stf.transform.rotation.w = 1.0
    static_tf.sendTransform(stf)

    proc = subprocess.Popen([
        binary, '--ros-args',
        '-p', 'input_scan_topic:=/scan',
        '-p', 'use_footprint_polygon:=true',
        '-p', 'cell_decay_sec:=15.0',
        '-p', 'decay_check_period_sec:=0.2',
        '-p', 'dynamic_memory_enable:=true',
        '-p', 'dynamic_decay_sec:=1.0',
        '-p', 'dynamic_min_hold_sec:=1.0',
        '-p', 'dynamic_input_topic:=/scan_dynamic',
        '-p', 'trace_period_sec:=0.5',
    ], env=dict(os.environ, ROS_DOMAIN_ID=domain, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST'),
       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    out = ''
    try:
        deadline = time.monotonic() + 5.0
        while (scan_pub.get_subscription_count() == 0
               or dyn_pub.get_subscription_count() == 0
               or node.count_subscribers('/tf') == 0
               or node.count_subscribers('/tf_static') == 0) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            time.sleep(0.02)
        assert scan_pub.get_subscription_count() > 0
        assert node.count_subscribers('/tf') > 0
        static_tf.sendTransform(stf)
        time.sleep(0.1)
        base_sec = time.time()

        def send_frame(sec_offset, beam0_r, beam0_dyn, beam10_r=math.inf, beam10_dyn=0.0, rx=0.0):
            ts = base_sec + sec_offset
            stamp = rclpy.time.Time(seconds=int(ts),
                                    nanoseconds=int((ts % 1.0) * 1e9))
            for dt in (-0.05, 0.0, 0.05):
                t_tf = ts + dt
                tf_msg = TransformStamped()
                tf_msg.header.stamp = rclpy.time.Time(
                    seconds=int(t_tf), nanoseconds=int((t_tf % 1.0) * 1e9)).to_msg()
                tf_msg.header.frame_id = 'odom'
                tf_msg.child_frame_id = 'odin1_base_link'
                tf_msg.transform.translation.x = float(rx)
                tf_msg.transform.rotation.w = 1.0
                dyn_tf.sendTransform(tf_msg)
            time.sleep(0.03)

            scan = LaserScan()
            scan.header.stamp = stamp.to_msg()
            scan.header.frame_id = 'odin1_base_link'
            scan.angle_min = 0.0
            scan.angle_max = 0.25
            scan.angle_increment = 0.25
            scan.range_min = 0.2
            scan.range_max = 5.0
            ranges = [float(beam0_r), float(beam10_r)]
            scan.ranges = ranges

            dyn = LaserScan()
            dyn.header = scan.header
            dyn.ranges = ranges
            intensities = [float(beam0_dyn), float(beam10_dyn)]
            dyn.intensities = intensities
            count = len(maps)
            wait_end = time.monotonic() + 3.0
            while len(maps) <= count and time.monotonic() < wait_end:
                for dt in (-0.05, 0.0, 0.05):
                    t_tf = ts + dt
                    tf_msg = TransformStamped()
                    tf_msg.header.stamp = rclpy.time.Time(
                        seconds=int(t_tf), nanoseconds=int((t_tf % 1.0) * 1e9)).to_msg()
                    tf_msg.header.frame_id = 'odom'
                    tf_msg.child_frame_id = 'odin1_base_link'
                    tf_msg.transform.translation.x = float(rx)
                    tf_msg.transform.rotation.w = 1.0
                    dyn_tf.sendTransform(tf_msg)
                dyn_pub.publish(dyn)
                time.sleep(0.02)
                scan_pub.publish(scan)
                for _ in range(3):
                    executor.spin_once(timeout_sec=0.02)
            assert len(maps) > count

        def cell_val(msg, wx, wy):
            mx = int((wx - msg.info.origin.position.x) / msg.info.resolution)
            my = int((wy - msg.info.origin.position.y) / msg.info.resolution)
            return msg.data[my * msg.info.width + mx]

        # 1) Observe free along beam 0, then hit dynamic target at (1.0, 0.0) at t=0.2s
        send_frame(0.0, math.inf, 0.0)
        send_frame(0.2, 1.0, 1.0)
        assert cell_val(maps[-1], 1.0, 0.0) == 100

        # 2) At t=0.4s target moves to beam 1 while beam 0 is occluded at 0.5m
        p2x, p2y = 1.0 * math.cos(0.25), 1.0 * math.sin(0.25)
        send_frame(0.4, 0.5, 0.0, beam10_r=1.0, beam10_dyn=1.0)
        assert cell_val(maps[-1], p2x, p2y) == 100

        # 3) At t=1.8s (> dynamic_min_hold_sec=1.0s): vacated (1.0, 0.0) is cleared,
        #    while unvacated (p2x, p2y) and static (0.5, 0.0) remain occupied
        send_frame(1.8, 0.5, 0.0, beam10_r=0.5, beam10_dyn=0.0)
        assert cell_val(maps[-1], 1.0, 0.0) == -1
        assert cell_val(maps[-1], p2x, p2y) == 100
        assert cell_val(maps[-1], 0.5, 0.0) == 100

        # 4) Advance robot rx=6.0 (> recenter_threshold_m=5.0) to trigger maybeRecenter;
        #    verify both dynamic and static occupied cells are preserved
        send_frame(2.0, math.inf, 0.0, beam10_r=math.inf, beam10_dyn=0.0, rx=6.0)
        assert cell_val(maps[-1], p2x, p2y) == 100
        assert cell_val(maps[-1], 0.5, 0.0) == 100
    finally:
        proc.terminate()
        try:
            out, _ = proc.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, _ = proc.communicate()
        executor.shutdown()
        node.destroy_node()
        ctx.shutdown()
    assert '[sliding_map] memory trace:' in out
