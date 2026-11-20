"""ROS-agnostic state and map rendering for the web console."""

import array
import hashlib
import math
import os
import struct
import threading
import time
import zlib

import yaml

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


def _read_pgm(path):
    with open(path, "rb") as fh:
        data = fh.read()
    idx = 0
    n = len(data)
    tokens = []
    while len(tokens) < 4 and idx < n:
        while idx < n and data[idx] in b" \t\r\n":
            idx += 1
        if idx >= n:
            break
        if data[idx] == ord("#"):
            while idx < n and data[idx] not in b"\r\n":
                idx += 1
            continue
        start = idx
        while idx < n and data[idx] not in b" \t\r\n#":
            idx += 1
        tokens.append(data[start:idx].decode("ascii"))
    if len(tokens) < 4 or tokens[0] not in ("P5", "P2"):
        raise ValueError("Unsupported PGM header in {}".format(path))
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if width <= 0 or height <= 0 or maxval <= 0 or maxval > 255:
        raise ValueError("Invalid PGM dimensions in {}".format(path))
    if magic == "P5":
        if idx < n and data[idx] in b" \t\r\n":
            idx += 1
        pixels = data[idx:idx + width * height]
        if len(pixels) < width * height:
            raise ValueError("Truncated PGM payload in {}".format(path))
    else:
        body_lines = [ln.split(b"#", 1)[0] for ln in data[idx:].splitlines()]
        vals = [int(tok) for ln in body_lines for tok in ln.split()]
        if len(vals) < width * height:
            raise ValueError("Truncated ASCII PGM payload in {}".format(path))
        pixels = bytes(vals[:width * height])
    return width, height, maxval, pixels


def load_pgm_yaml(yaml_path):
    """Load a ROS map_server .yaml + .pgm pair into (width, height, resolution, origin_x, origin_y, cells)."""
    try:
        with open(yaml_path, "r", encoding="utf-8") as fh:
            meta = yaml.safe_load(fh) or {}
        img_rel = str(meta["image"])
        pgm_path = img_rel if os.path.isabs(img_rel) else os.path.join(os.path.dirname(yaml_path), img_rel)
        width, height, maxval, pixels = _read_pgm(pgm_path)
        res = float(meta["resolution"])
        origin = meta.get("origin") or [0.0, 0.0, 0.0]
        ox, oy = float(origin[0]), float(origin[1])
        negate = bool(int(meta.get("negate", 0)))
        occ_th = float(meta.get("occupied_thresh", 0.65))
        free_th = float(meta.get("free_thresh", 0.196))
        mode = str(meta.get("mode", "trinary")).strip().lower()
    except Exception:
        return None

    lut = bytearray(256)
    for p in range(256):
        shade = min(1.0, p / float(maxval))
        occ = shade if negate else (1.0 - shade)
        if mode == "raw":
            val = int(round(shade * 100.0))
            cell = val if 0 <= val <= 100 else -1
        elif occ > occ_th:
            cell = 100
        elif occ < free_th:
            cell = 0
        elif mode == "scale":
            cell = int(round(99.0 * (occ - free_th) / max(1e-6, occ_th - free_th)))
        else:
            cell = -1
        lut[p] = cell & 0xFF

    mapped = pixels.translate(bytes(lut))
    # PGM row 0 is top (y_max); OccupancyGrid row 0 is bottom (origin_y).
    flipped = b"".join(mapped[r * width:(r + 1) * width] for r in reversed(range(height)))
    cells = array.array("b")
    cells.frombytes(flipped)
    return width, height, res, ox, oy, cells


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
