#!/usr/bin/env python3
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

import copy
import json
import math
from collections import deque


class TemporalSpeckleFilter:
    def __init__(self, enabled=False, history_frames=5, min_hits=3,
                 sector_min_deg=30.0, sector_max_deg=50.0,
                 max_range_m=0.60, immediate_cluster_points=3,
                 angle_tolerance_deg=2.5, range_tolerance_m=0.15):
        self.enabled = bool(enabled)
        self.history_frames = max(1, int(history_frames))
        self.min_hits = min(self.history_frames, max(1, int(min_hits)))
        self.sector_min_deg = float(sector_min_deg)
        self.sector_max_deg = float(sector_max_deg)
        self.max_range_m = float(max_range_m)
        self.immediate_cluster_points = max(1, int(immediate_cluster_points))
        self.angle_tolerance_deg = float(angle_tolerance_deg)
        self.range_tolerance_m = float(range_tolerance_m)
        self._history = deque(maxlen=max(0, self.history_frames - 1))
        self._geometry = None

    def reset(self):
        self._history.clear()
        self._geometry = None

    def _candidate(self, distance, angle_deg, range_min):
        return (
            math.isfinite(distance)
            and distance >= range_min
            and distance < self.max_range_m
            and self.sector_min_deg <= abs(angle_deg) <= self.sector_max_deg
        )

    def process(self, ranges, angle_min, angle_increment, range_min):
        values = list(ranges)
        geometry = (len(values), round(float(angle_min), 8), round(float(angle_increment), 8))
        if geometry != self._geometry:
            self._history.clear()
            self._geometry = geometry
        if not self.enabled:
            self._history.clear()
            return values, {
                "enabled": False,
                "candidate_points": 0,
                "suppressed_points": 0,
                "immediate_points": 0,
                "temporal_points": 0,
            }

        angles = [math.degrees(angle_min + i * angle_increment) for i in range(len(values))]
        indices = [
            i for i, distance in enumerate(values)
            if self._candidate(distance, angles[i], range_min)
        ]
        clusters = []
        if indices:
            current = [indices[0]]
            for index in indices[1:]:
                previous = current[-1]
                if index == previous + 1 and abs(values[index] - values[previous]) <= self.range_tolerance_m:
                    current.append(index)
                else:
                    clusters.append(current)
                    current = [index]
            clusters.append(current)

        keep = set()
        immediate = 0
        temporal = 0
        for cluster in clusters:
            if len(cluster) >= self.immediate_cluster_points:
                keep.update(cluster)
                immediate += len(cluster)
                continue
            hits = 1
            for history_frame in self._history:
                matched = any(
                    abs(angles[index] - old_angle) <= self.angle_tolerance_deg
                    and abs(values[index] - old_range) <= self.range_tolerance_m
                    for index in cluster
                    for old_angle, old_range in history_frame
                )
                hits += int(matched)
            if hits >= self.min_hits:
                keep.update(cluster)
                temporal += len(cluster)

        suppressed = 0
        for index in indices:
            if index not in keep:
                values[index] = float("inf")
                suppressed += 1
        self._history.append([(angles[index], ranges[index]) for index in indices])
        return values, {
            "enabled": True,
            "candidate_points": len(indices),
            "suppressed_points": suppressed,
            "immediate_points": immediate,
            "temporal_points": temporal,
        }


def main():
    import rospy
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    rospy.init_node("scan_temporal_filter")
    filter_core = TemporalSpeckleFilter(
        enabled=rospy.get_param("~enable", False),
        history_frames=rospy.get_param("~history_frames", 5),
        min_hits=rospy.get_param("~min_hits", 3),
        sector_min_deg=rospy.get_param("~sector_min_deg", 30.0),
        sector_max_deg=rospy.get_param("~sector_max_deg", 50.0),
        max_range_m=rospy.get_param("~max_range_m", 0.60),
        immediate_cluster_points=rospy.get_param("~immediate_cluster_points", 3),
        angle_tolerance_deg=rospy.get_param("~angle_tolerance_deg", 2.5),
        range_tolerance_m=rospy.get_param("~range_tolerance_m", 0.15),
    )
    output_topic = rospy.get_param("~output_topic", "/scan")
    input_topic = rospy.get_param("~input_topic", "/scan_raw")
    scan_pub = rospy.Publisher(output_topic, LaserScan, queue_size=1)
    trace_pub = rospy.Publisher("/scan_temporal_filter/trace", String, queue_size=20)

    def callback(msg):
        filtered, stats = filter_core.process(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min
        )
        output = copy.deepcopy(msg)
        output.ranges = filtered
        scan_pub.publish(output)
        stats["stamp"] = msg.header.stamp.to_sec()
        trace_pub.publish(String(data=json.dumps(stats, separators=(",", ":"))))
        if stats["suppressed_points"]:
            rospy.loginfo_throttle(
                2.0,
                "[scan_temporal_filter] suppressed=%d candidate=%d immediate=%d temporal=%d",
                stats["suppressed_points"], stats["candidate_points"],
                stats["immediate_points"], stats["temporal_points"],
            )

    rospy.Subscriber(input_topic, LaserScan, callback, queue_size=1, tcp_nodelay=True)
    rospy.loginfo(
        "[scan_temporal_filter] enable=%s input=%s output=%s history=%d min_hits=%d "
        "sector=%.1f..%.1fdeg range<%.2fm immediate_cluster=%d",
        filter_core.enabled, input_topic, output_topic,
        filter_core.history_frames, filter_core.min_hits,
        filter_core.sector_min_deg, filter_core.sector_max_deg,
        filter_core.max_range_m, filter_core.immediate_cluster_points,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
