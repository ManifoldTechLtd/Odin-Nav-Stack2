# Inspection Perception (Florence-2)

**English** | [中文](README.zh-CN.md)

The edge visual inspection and VLM query module (**Florence-2** / **SmolVLM** / **Moondream2**) is maintained in a standalone open-source repository so its Python/PyTorch environment stays decoupled from this ROS workspace:

- **Repository:** [https://github.com/ManifoldTechLtd/Odin-perception](https://github.com/ManifoldTechLtd/Odin-perception)

---

## Architecture & Dataflow with ONS2

`Odin-perception` connects to the navigation stack via two processes over HTTP:

```text
odin_ros_driver ──► /odin1/image/undistorted
                            │
              ros1/ros2 capture node (:8730) ──HTTP──► perception-run (Florence-2)
                            ▲                                   │
                            └───────── POST /annotate ◄─────────┘
                            │
              /inspection/annotated/compressed ──► Foxglove / RViz / rosbridge
```

1. **Navigation-side prerequisite:** Enable undistorted RGB output in the Odin driver's `control_command.yaml` (`sendrgb: 1` and `sendrgbundistort: 1`), and verify with:
   ```bash
   rostopic hz /odin1/image/undistorted        # ROS 1
   ros2 topic hz /odin1/image/undistorted      # ROS 2
   ```
2. **Perception-side setup:** See the *Integrating with Odin-Nav-Stack* section in the [`Odin-perception` README](https://github.com/ManifoldTechLtd/Odin-perception).

---

## How to Clone into `perception/` (Without Nested `perception/Odin-perception/` Directories)

If you want to place the `Odin-perception` source code directly inside this `perception/` directory **without** creating an extra nested `perception/Odin-perception/` folder, run either of the following from the **root of this repository** (`Odin-Nav-Stack2/`):

### Option 1: Replace `perception/` directly (Recommended)

```bash
# Run from the Odin-Nav-Stack2 repository root:
rm -rf perception
git clone https://github.com/ManifoldTechLtd/Odin-perception.git perception
```

### Option 2: Clone alongside `Odin-Nav-Stack2` as a sibling directory

```bash
cd ..
git clone https://github.com/ManifoldTechLtd/Odin-perception.git
```
