# Q9000 Map Scanning & Processing Guide

**English** | [中文](Q9000-tutorial.zh-CN.md)

> [!NOTE]
> Although this guide uses the **Unitree Go2** as a concrete example for file transfer and mounting height parameters, the Q9000 scanning, map export, and PCD-to-PGM grid conversion workflow is **completely platform-agnostic**. It applies equally to any robot running ONS2 (such as ASTRALL SW01, Dobot Lumo, or custom mobile platforms) regardless of the specific robot model.

This document walks through the complete workflow from scanning an environment with a **Q9000**, exporting the point-cloud and relocalization maps in **MindCloud Studio**, and converting them into a 2D occupancy grid map ready for autonomous navigation.

---

## 1. Connect the Q9000 to Your Workstation

Connect a Type-C to USB-A or Type-C to Type-C data cable between the USB-C port on the back of the [Q9000](https://www.manifoldtech.cn/products/q9000) head and your computer. Once recognized, a removable USB drive will appear on your computer containing the recorded project folders.

![Q9000 bottom interface ports](https://i.ibb.co/prvWBS9C/09.png)

> [!TIP]
> A workstation or laptop with a dedicated GPU is recommended. Ultrabooks without a discrete GPU will process point clouds significantly more slowly.

---

## 2. Open the Project in MindCloud Studio

Launch **MindCloud Studio** on your computer and follow the steps below. (Contact Manifold Tech support to obtain the appropriate MindCloud Studio installer and license.)

1. In the top-left menu bar, click **File → New Project → Single Point Cloud Project** (`新建工程 -> 单点云工程`):

   ![MindCloud Studio File -> New Single Point Cloud Project](https://i.ibb.co/hFMQJbY9/10.png)

2. Select your recorded project folder from the Q9000 USB drive and open the `.lx` project file.  
   *(Tip: Copy the project folder from the Q9000 USB drive to your local disk beforehand so the Q9000 battery does not run out during long exports.)*

   ![Select the .lx project file](https://i.ibb.co/LXV6SR7b/11.png)

3. Configure the processing options in the pop-up dialog:

   ![Single Point Cloud Project settings](https://i.ibb.co/y3R8BKh/12.png)

4. Wait for loading and processing to finish. If you do not need to archive a high-precision digital twin asset, enabling **Loop Closure Detection** (`回环检测`) and **Moving Object Removal** (`运动物体滤除`) is sufficient—you can leave **Bundle Adjustment** (`平差优化`) unchecked for faster map generation.

---

## 3. Export Map Files

Once processing completes, export two files:
- **`.pcd`** — 3D point-cloud map (used to generate the 2D `.pgm` + `.yaml` occupancy grid for global planning)
- **`.bin`** — Odin relocalization map

![Processed 3D point cloud in MindCloud Studio](https://i.ibb.co/8gH2mPm4/13.png)

### 3.1 Export the `.pcd` Point Cloud

1. In the left project tree, right-click the point cloud item and select **Save** (`保存`):

   ![Right-click point cloud and click Save](https://i.ibb.co/Ft90W1Y/14.png)

2. In the save dialog, set **Save as type** to `PCD cloud (*.pcd)` and save it to your local folder:

   ![Save as PCD cloud (*.pcd)](https://i.ibb.co/r28KZbR4/15.png)

### 3.2 Export the `.bin` Relocalization Map

1. In the top menu bar, click **Industry Features → Navigation Map** (`行业功能 -> 导航地图`):

   ![Industry Features -> Navigation Map](https://i.ibb.co/d0gxffrg/16.png)

2. Wait for the navigation map export to complete (larger environments take longer to export):

   ![Navigation map generation progress](https://i.ibb.co/WptMmFtx/17.png)

3. Once finished, the exported `.bin` file is saved inside the `export_map/` subfolder of your project directory:

   ![export_map folder inside the project directory](https://i.ibb.co/XZC96SFV/18.png)

---

## 4. Convert `.pcd` to a 2D Grid Map (`.pgm` + `.yaml`)

> [!NOTE]
> The commands below use a Unitree Go2 (`192.168.123.18`, user `unitree`, ROS 1 workspace `ros_ws/`) as an example. Replace the IP address, username, workspace path (`ros_ws` or `ros2_ws`), and `--lidar-height` with the values for your robot.

### 4.1 Copy `.pcd` and `.bin` to the Robot

> [!IMPORTANT]
> The `customer_file/` directory at the repository root (`/path/to/base_inspection/customer_file/`) is ignored by Git and **does not exist in a fresh clone**. You must **create it manually** before copying `.bin` files into it:
> ```bash
> mkdir -p /path/to/base_inspection/customer_file
> ```

Transfer the exported `.pcd` and `.bin` files to your robot using either a USB flash drive or `scp`:

**Option A — Via USB Flash Drive:**

```bash
# Ensure customer_file/ exists at the repository root
mkdir -p /path/to/base_inspection/customer_file

# Enter the USB drive mount directory
cd /media/Upan

# Move the .pcd and .bin files to the navigation stack directories
mv parkinglot.pcd /path/to/base_inspection/ros_ws/src/map_planner/maps/parkinglot.pcd
mv parkinglot.bin /path/to/base_inspection/customer_file/parkinglot.bin

# Verify the files are in place
ls /path/to/base_inspection/ros_ws/src/map_planner/maps/parkinglot.pcd
ls /path/to/base_inspection/customer_file/parkinglot.bin
```

**Option B — Via `scp` over the Network:**

```bash
# First, create customer_file/ on the robot if it does not exist yet:
ssh unitree@192.168.123.18 "mkdir -p /path/to/base_inspection/customer_file"

# Copy the point cloud (.pcd) to map_planner/maps/
scp parkinglot.pcd \
    unitree@192.168.123.18:/path/to/base_inspection/ros_ws/src/map_planner/maps/

# Copy the relocalization map (.bin) to customer_file/
scp parkinglot.bin \
    unitree@192.168.123.18:/path/to/base_inspection/customer_file/
```

### 4.2 Run `pcd_to_pgm.py`

Run `scripts/map_processing/pcd_to_pgm.py` (or copy it into `map_planner/maps/`) to project a height slice of the 3D `.pcd` point cloud into a 2D occupancy grid:

```bash
cd /path/to/base_inspection/ros_ws/src/map_planner/maps

python3 /path/to/base_inspection/scripts/map_processing/pcd_to_pgm.py parkinglot.pcd \
    -o parkinglot \
    -r 0.05 \
    --lidar-height 0.545 \
    --slice-above 0.3 \
    --slice-below 0.2
```

| Argument | Default | Description |
|---|---|---|
| `parkinglot.pcd` | *(required)* | Input `.pcd` file path |
| `-o` | Input stem | Output filename prefix; generates `<prefix>.pgm` and `<prefix>.yaml`. **Note: the `.pgm`, `.yaml`, and `.bin` files should share the same base name.** |
| `-r` | `0.05` | Grid resolution in meters/cell (`0.05` recommended; increase to `0.1+` for very large maps) |
| `--lidar-height` | `0.545` | Sensor mounting height above ground in meters (`0.545` is the default for Go2; adjust for your robot) |
| `--slice-above` | `0.3` | Height window above the sensor (m) retained as obstacles |
| `--slice-below` | `0.2` | Height window below the sensor (m) retained as obstacles |

> How to pick the two height windows from your robot's height and sensor mount is covered in
> [Tuning Guide §3.1b Point-Cloud Height Slicing](../TUNING_GUIDE.md).

### 4.3 Verify the Generated Files

The script produces two files in `map_planner/maps/`:

```text
parkinglot.pgm    # 2D occupancy grid image
parkinglot.yaml   # ROS map_server metadata file
```

You can inspect `parkinglot.pgm` directly or clean up noise in an image editor such as GIMP (Section 4.4).

### 4.4 Optional Manual Map Cleanup in GIMP

If the raw slice contains transient clutter or artifacts you want to erase manually, open the `.pgm` file in **GIMP**:

```bash
sudo apt install gimp
gimp
```

Drag `parkinglot.pgm` into GIMP:

![Opening the .pgm map in GIMP](https://i.ibb.co/nqysk3Ws/27.png)

- **Zoom / Pan:** Hold `Ctrl` + scroll wheel to zoom; hold the middle mouse button and drag to pan.
- **Pencil Tool:** Select the **Pencil** tool in the top-left toolbar (set **Hardness = 100**), and pick pure **White** or **Black** from the color palette:
  - **White** = free traversable space
  - **Black** = obstacle
  - **Gray** = unknown / unexplored area

![GIMP Pencil tool settings](https://i.ibb.co/1jvJ9k6/28.png)

- **Export:** Click **File → Export As...**, export as `.pgm` (raw/binary PGM), and overwrite `parkinglot.pgm` in `map_planner/maps/` (keep `parkinglot.yaml` unchanged).

---

## 5. Launch Navigation with `--map=<map_name>`

Once the map files are ready, pass `--map=<map_name>` to the launcher script:

```bash
bash /path/to/base_inspection/scripts/ons2_go2.sh --map=<map_name>    # e.g., --map=parkinglot
# On ROS 2 (SW01): bash /path/to/base_inspection/scripts/ons2_sw01.sh --map=parkinglot
# On Dobot Lumo:   bash /path/to/base_inspection/scripts/ons2_dobot_lumo.sh --map=parkinglot
```

Here `--map=<map_name>` selects the built map by its base name. Every map consists of **three files that must share the exact same base name** (for example, `parkinglot.yaml`, `parkinglot.pgm`, and `parkinglot.bin`):

| File | Required Directory | Notes |
|---|---|---|
| `<map_name>.yaml` | `/path/to/base_inspection/ros_ws/src/map_planner/maps/`<br/>*(or `ros2_ws/src/map_planner/maps/` on ROS 2)* | 2D map metadata (`resolution`, `origin`, thresholds) |
| `<map_name>.pgm` | `/path/to/base_inspection/ros_ws/src/map_planner/maps/`<br/>*(or `ros2_ws/src/map_planner/maps/` on ROS 2)* | 2D occupancy grid image |
| `<map_name>.bin` | `/path/to/base_inspection/customer_file/` | Odin1 3D relocalization map (**note:** `customer_file/` does not exist by default in a fresh clone—create it manually with `mkdir -p /path/to/base_inspection/customer_file`) |

For full mission control and patrol instructions after startup, see [01 · Quick Start](../usage/01_quick_start.md).
