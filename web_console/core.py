"""ROS-agnostic state and map rendering for the web console."""

import hashlib
import math
import struct
import threading
import time
import zlib

UNKNOWN_GRAY = 205
OCCUPIED_THRESHOLD = 65


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _build_occupancy_lut():
    lut = bytearray(256)
    for index in range(256):
        value = index - 256 if index >= 128 else index
        if value < 0:
            gray = UNKNOWN_GRAY
        elif value >= OCCUPIED_THRESHOLD:
            gray = 0
        else:
            gray = int(round(255 * (1.0 - value / 100.0)))
        lut[index] = gray
    return bytes(lut)


OCCUPANCY_LUT = _build_occupancy_lut()


def occupancy_to_png(cells, width, height):
    raw = bytes(memoryview(cells).cast("B")).translate(OCCUPANCY_LUT)
    rows = [raw[r * width:(r + 1) * width] for r in reversed(range(height))]
    return encode_png_gray(b"".join(rows), width, height)


def encode_png_gray(raw, width, height):
    stride = width
    filtered = bytearray()
    for r in range(height):
        filtered.append(0)
        filtered.extend(raw[r * stride:(r + 1) * stride])

    def chunk(ctype, body):
        return struct.pack(">I", len(body)) + ctype + body + struct.pack(
            ">I", zlib.crc32(ctype + body) & 0xFFFFFFFF)

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(bytes(filtered)))
            + chunk(b"IEND", b""))


class ConsoleState:
    def __init__(self):
        self._lock = threading.Lock()
        self._pose = None
        self._pose_stamp = 0.0
        self._map = None
        self._map_png = None
        self._map_version = 0
        self._map_key = None

    def set_pose(self, x, y, yaw):
        with self._lock:
            self._pose = (float(x), float(y), float(yaw))
            self._pose_stamp = time.time()

    def clear_pose(self):
        with self._lock:
            self._pose = None

    def set_map(self, width, height, resolution, origin_x, origin_y, cells):
        key = (width, height, float(resolution), float(origin_x), float(origin_y),
               hashlib.sha1(bytes(memoryview(cells).cast("B"))).digest())
        with self._lock:
            if key == self._map_key:
                return
            self._map_key = key
        png = occupancy_to_png(cells, width, height)
        with self._lock:
            self._map_version += 1
            self._map = {"version": self._map_version, "width": width, "height": height,
                         "resolution": float(resolution),
                         "origin": {"x": float(origin_x), "y": float(origin_y)}}
            self._map_png = png

    def map_png(self):
        with self._lock:
            return self._map_version, self._map_png

    def snapshot(self):
        now = time.time()
        with self._lock:
            pose = None
            if self._pose is not None:
                x, y, yaw = self._pose
                pose = {"x": x, "y": y, "yaw": yaw, "yaw_deg": math.degrees(yaw),
                        "age_sec": max(0.0, now - self._pose_stamp)}
            return {"pose": pose, "map": self._map, "server_time": now}
