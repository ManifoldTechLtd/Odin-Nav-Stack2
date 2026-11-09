# YOLOv5 Weights

This folder holds the YOLOv5 weights used by `yolo_ros`. The detector loads `yolov5s.pt` from here by default. The file is about 14 MB and is not distributed with this repository.

## Download

Download the official Ultralytics YOLOv5 release:

```bash
cd ros_ws/src/yolo_ros/scripts/models
curl -L -o yolov5s.pt \
  https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt
```

You can also obtain it from another Ultralytics release channel; keep the file name `yolov5s.pt`.

## Dependencies

`yolo_detector.py` loads the `yolov5/` submodule from the repository root by relative path. Initialize it before first use.

Run the following command **from the repository root** (the directory that contains `.gitmodules` and the `yolov5/` submodule):

```bash
cd /path/to/Odin-Nav-Stack2
git submodule update --init yolov5
```

Do not run it from `ros_ws/src/yolo_ros/scripts/models`; the `yolov5` path is relative to the repository root.

The `yolov5` submodule is AGPL-3.0. Follow Ultralytics' license terms when using the code and weights.

## Custom weights

Place your trained weights in this folder and update the model path in `yolo_detector.py` or the launch file. The default file name is `yolov5s.pt`.
