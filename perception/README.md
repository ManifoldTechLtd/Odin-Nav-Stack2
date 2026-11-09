# Inspection Perception (Florence-2)

**English** | [中文](README.zh-CN.md)

The edge visual inspection and VLM query module (**Florence-2** / **SmolVLM** / **Moondream2**) is maintained in a standalone open-source repository so its Python/PyTorch environment stays decoupled from this ROS workspace:

- **Repository:** [https://github.com/ManifoldTechLtd/Odin-perception](https://github.com/ManifoldTechLtd/Odin-perception)

> **This is an optional module.** ONS2 navigates, plans, and avoids obstacles without it.
> Adding perception never requires recompiling the stack, editing launch files, or touching
> navigation parameters — it attaches as two side-car processes over HTTP on port 8730.

**Decide whether you need it:**

| Add Odin-perception when… | Skip it when… |
|---|---|
| Patrols must emit **pass/fail verdicts** against a checklist (valve state, gauge reading, indicator color, nameplate OCR) | The robot only needs to **reach waypoints** |
| You want **annotated frames** (detection boxes + verdict text) on the camera topic | A human reviews the raw video stream |
| Your camera publishes RGB (`sendrgb: 1` + `sendrgbundistort: 1`) | No RGB stream is available on the platform |

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
2. **Perception-side setup:** See the *Integrating with Odin-Nav-Stack2* section in the [`Odin-perception` README](https://github.com/ManifoldTechLtd/Odin-perception).

---

## How to Install

### Option A — One command (recommended)

```bash
# Run from the Odin-Nav-Stack2 repository root:
bash perception/setup.sh
```

This clones `Odin-perception` into `~/odin-perception` (**outside** this repo, so this guide
and the `perception/` folder stay intact), creates an isolated venv, installs pinned
dependencies, downloads the Florence-2 weights and runs a self-check.

Useful flags:

| Flag | Effect |
|---|---|
| `--tag v0.1.0` | Pin a release (recommended for production) |
| `--dir <path>` | Install somewhere else |
| `--cpu` | Force the CPU torch wheel (no GPU platform) |
| `--check-only` | Self-check only, no install |
| `--skip-weights` | Skip the 888 MB download (offline devices) |

The script is an **orchestrator**: it calls `Odin-perception`'s own `scripts/setup.sh`,
`scripts/download_models.sh` and `scripts/doctor.py`, so installation logic never gets
duplicated here.

### Option B — Manual clone into `perception/` (overwrites this guide)

If you would rather keep the source inside this repo without a nested
`perception/Odin-perception/` folder:

```bash
# Run from the Odin-Nav-Stack2 repository root:
rm -rf perception
git clone https://github.com/ManifoldTechLtd/Odin-perception.git perception
```

> ⚠️ This **replaces** the `perception/` directory, deleting the two guide files here.
> Keep a copy if you still want them.

### Option C — Clone as a sibling directory

```bash
cd ..
git clone https://github.com/ManifoldTechLtd/Odin-perception.git
```
