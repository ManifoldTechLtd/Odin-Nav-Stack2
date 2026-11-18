import array
import math
import struct
import zlib

import pytest

from web_console import core


def decode_png_gray(data):
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    pos = 8
    chunks = {}
    idat = b""
    while pos < len(data):
        length, = struct.unpack(">I", data[pos:pos + 4])
        ctype = data[pos + 4:pos + 8]
        body = data[pos + 8:pos + 8 + length]
        if ctype == b"IDAT":
            idat += body
        else:
            chunks[ctype] = body
        pos += 12 + length
    width, height, depth, color, _, _, _ = struct.unpack(">IIBBBBB", chunks[b"IHDR"])
    assert (depth, color) == (8, 0)
    raw = zlib.decompress(idat)
    rows = []
    stride = width + 1
    for r in range(height):
        row = raw[r * stride:(r + 1) * stride]
        assert row[0] == 0
        rows.append(list(row[1:]))
    return width, height, rows


def test_yaw_from_quaternion():
    assert core.yaw_from_quaternion(0, 0, 0, 1) == pytest.approx(0.0)
    half = math.pi / 4
    assert core.yaw_from_quaternion(0, 0, math.sin(half), math.cos(half)) == pytest.approx(math.pi / 2)


def test_occupancy_lut_maps_signed_cells_to_gray():
    lut = core.OCCUPANCY_LUT
    assert lut[(-1) & 0xFF] == core.UNKNOWN_GRAY
    assert lut[0] == 255
    assert lut[core.OCCUPIED_THRESHOLD] == 0
    assert lut[100] == 0
    assert 100 < lut[50] < 160


def test_encode_png_gray_roundtrip():
    png = core.encode_png_gray(bytes([0, 255, 128, 64]), 2, 2)
    assert decode_png_gray(png) == (2, 2, [[0, 255], [128, 64]])


def test_occupancy_to_png_flips_rows_so_origin_row_is_bottom():
    # OccupancyGrid row 0 is the bottom (y = origin_y); PNG row 0 is the top.
    cells = array.array("b", [100, 0, 0, 0])  # occupied at bottom-left cell
    width, height, rows = decode_png_gray(core.occupancy_to_png(cells, 2, 2))
    assert rows[1][0] == 0
    assert rows[0] == [255, 255]


def test_state_snapshot_pose():
    state = core.ConsoleState()
    assert state.snapshot()["pose"] is None
    state.set_pose(1.0, 2.0, math.pi / 2)
    pose = state.snapshot()["pose"]
    assert (pose["x"], pose["y"]) == (1.0, 2.0)
    assert pose["yaw_deg"] == pytest.approx(90.0)
    assert 0.0 <= pose["age_sec"] < 1.0
    state.clear_pose()
    assert state.snapshot()["pose"] is None


def test_state_map_version_and_png():
    state = core.ConsoleState()
    assert state.map_png() == (0, None)
    cells = array.array("b", [0, 0, 0, 0])
    state.set_map(2, 2, 0.05, -1.0, -3.0, cells)
    version, png = state.map_png()
    assert version == 1
    assert decode_png_gray(png)[:2] == (2, 2)
    meta = state.snapshot()["map"]
    assert meta == {"version": 1, "width": 2, "height": 2, "resolution": 0.05,
                    "origin": {"x": -1.0, "y": -3.0}}
    state.set_map(2, 2, 0.05, -1.0, -3.0, cells)
    assert state.map_png()[0] == 1
    state.set_map(2, 2, 0.05, -1.0, -3.0, array.array("b", [100, 0, 0, 0]))
    assert state.map_png()[0] == 2
