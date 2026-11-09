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

"""Extract a frozen geometry fixture, not a live sensor/TF timing replay."""
import argparse
import json
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from mcap.reader import make_reader
from nav_interfaces.msg import MissionStatus
from rclpy.serialization import deserialize_message, serialize_message
from tf2_msgs.msg import TFMessage
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--request-time", type=float, default=83.616)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with args.bag.open("rb") as stream:
        reader = make_reader(stream)
        t0 = reader.get_summary().statistics.message_start_time
        for _, _, message in reader.iter_messages(topics=["/map"]):
            (args.output / "map.cdr").write_bytes(message.data)
            break
        latest = None
        for _, _, message in reader.iter_messages(
                topics=["/sliding_map"], end_time=t0 + int(args.request_time * 1e9)):
            latest = message
        assert latest is not None
        (args.output / "fake.cdr").write_bytes(latest.data)
        transforms = {}
        for _, _, message in reader.iter_messages(topics=["/tf_static"]):
            for transform in deserialize_message(message.data, TFMessage).transforms:
                transforms[(transform.header.frame_id, transform.child_frame_id)] = transform
        for _, _, message in reader.iter_messages(
                topics=["/tf"], end_time=t0 + int(args.request_time * 1e9)):
            for transform in deserialize_message(message.data, TFMessage).transforms:
                transforms[(transform.header.frame_id, transform.child_frame_id)] = transform
        for transform in transforms.values():
            transform.header.stamp.sec = 0
            transform.header.stamp.nanosec = 0
        tf = TFMessage()
        tf.transforms = list(transforms.values())
        (args.output / "tf.cdr").write_bytes(serialize_message(tf))
        goal = PoseStamped()
        goal.header.frame_id = "map"
        for _, _, message in reader.iter_messages(
                topics=["/waypoint_manager/status"],
                start_time=t0 + int(args.request_time * 1e9)):
            status = deserialize_message(message.data, MissionStatus)
            if status.current_target.position.x > 100:
                goal.pose = status.current_target
                break
        assert goal.pose.position.x > 100
        (args.output / "goal.cdr").write_bytes(serialize_message(goal))
    customer_file = Path(__file__).resolve().parents[1] / "config/nav_params.yaml"
    customer = yaml.safe_load(customer_file.read_text())
    parameters = {k: customer[k] for k in [
        "inflation_radius", "obstacle_cost_safe_distance", "line_of_sight_safe_distance",
        "smoothing_iterations", "smoothing_max_deviation_m"]}
    parameters.update(dict(obstacle_threshold=50, sliding_map_max_age_sec=2.0,
                           obstacle_cost_weight=8.0, turn_penalty_weight=0.5,
                           use_global_memory=False, path_initial_heading_tol_deg=45.0,
                           path_protect_initial_m=1.0, path_validate_enable=True,
                           path_min_clear_m=0.10))
    (args.output / "params.yaml").write_text(yaml.safe_dump({
        "/**": {"ros__parameters": parameters}}))
    print(json.dumps(dict(bag=str(args.bag), request_time=args.request_time,
                          goal=[goal.pose.position.x, goal.pose.position.y],
                          tf_edges=len(transforms)), indent=2))


if __name__ == "__main__":
    main()
