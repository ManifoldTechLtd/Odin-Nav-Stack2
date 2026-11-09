"""Lumo navigation input validity (no ROS imports, no control side effects)."""
import math
import threading
import time


class NavigationInputs:
    def __init__(self, max_age=0.75):
        self.max_age = max_age
        self.lock = threading.Lock()
        self.odom = None
        self.scan = None
        self.fault = ""

    def update_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v = msg.twist.twist.linear
        pos, vel = (p.x, p.y, p.z), (v.x, v.y, v.z)
        quat = (q.x, q.y, q.z, q.w)
        stamp = msg.header.stamp.to_sec()
        with self.lock:
            if not all(math.isfinite(x) for x in pos + vel + quat + (stamp,)):
                self.fault = "ODOM_NONFINITE"
                return
            # Sanity bounds for this local-map Lumo deployment, not motion limits.
            if max(abs(x) for x in pos) > 10000 or sum(x*x for x in vel) > 25:
                self.fault = "ODOM_IMPLAUSIBLE"
                return
            if not 0.9 <= sum(x*x for x in quat) <= 1.1:
                self.fault = "ODOM_QUATERNION"
                return
            if self.odom:
                dt = stamp - self.odom[0]
                if dt < 0:
                    self.fault = "ODOM_TIME_REVERSED"
                    return
                if dt == 0:
                    return  # duplicates do not refresh freshness
                distance = math.sqrt(sum((a-b)**2 for a, b in zip(pos, self.odom[1])))
                if distance > 0.1 + 5.0 * dt:
                    self.fault = "ODOM_JUMP"
                    return
            self.odom = (stamp, pos, time.monotonic())

    def update_scan(self, msg):
        hits = sum(math.isfinite(r) and msg.range_min <= r < msg.range_max
                   for r in msg.ranges)
        stamp = msg.header.stamp.to_sec()
        with self.lock:
            if self.scan and stamp == self.scan[0]:
                return
            self.scan = (stamp, hits, time.monotonic())

    def status(self, now=None):
        now = time.time() if now is None else now
        with self.lock:
            if self.fault:
                return self.fault  # pose faults require explicit restart after recovery
            for name, sample in (("ODOM", self.odom), ("SCAN", self.scan)):
                if sample is None:
                    return "NO_" + name
                age = now - sample[0]
                if (not math.isfinite(age) or not -0.1 <= age <= self.max_age
                        or time.monotonic() - sample[2] > self.max_age):
                    return name + "_STALE"
            # Conservative indoor admission: an all-empty scan is not evidence
            # of observed free space. Do not treat Inf/NaN as a clear corridor.
            if self.scan[1] == 0:
                return "NO_SCAN_HITS"
            return "OK"
