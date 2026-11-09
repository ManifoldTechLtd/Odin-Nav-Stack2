nav# yolo_ros

ROS 1 YOLOv5 detection and object-query package.

## What it does

- Detects objects in the Odin camera image with YOLOv5 (`yolo_detector.py`) and publishes 2D/3D detections.
- Finds a requested object in the latest detections and sends a navigation goal (`object_query_node.py`).
- Publishes the camera/lidar TF chain (`tf_relay_node.py`).
- Optional `auto_nav_mode`: a simple visual-servo controller that publishes `/cmd_vel` toward the nearest detected object.

## Current limitations

- Per-frame detection only: no tracker, no stable object ID, no identity across frames.
- `auto_nav_mode` is a nearest-box approach controller, not target following.
- Object query uses the latest detection snapshot; it does not keep following the object after sending a goal.

## In development

- Target tracking/follow is not implemented in this package. The separate `perception/inspection_perception/follow/` package provides YOLOv8 + ByteTrack tracking; its default profile is M1 (perception/ranging only), and the M2 goal-publishing path is not enabled by default and has not been validated end to end.
- YOLOv5 weights are not distributed with the repository; see `scripts/models/README.md`.

## Contents

| File | Purpose |
| --- | --- |
| `launch/yolo_detection.launch` | Launch entry (`roslaunch yolo_ros yolo_detection.launch`) |
| `scripts/yolo_detector.py` | YOLOv5 detection node |
| `scripts/tf_relay_node.py` | TF relay node |
| `scripts/object_query_node.py` | Object query node |
| `scripts/models/` | Detection weights directory; `yolov5s.pt` is not distributed with the repository, see `scripts/models/README.md` |

## Dependencies

- The `yolov5/` submodule at the repository root (`git submodule update --init yolov5`). `yolo_detector.py` loads it via a relative path.
- This package is not the YOLO used by `perception/inspection_perception/follow/` (VLM follow). That package uses Ultralytics YOLOv8 + ByteTrack; see `perception/configs/tasks/follow_yellow_person.yaml`.
