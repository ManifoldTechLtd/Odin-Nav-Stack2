# map_processing — Map & Point Cloud Processing Pipeline

This directory contains tools for converting **point clouds into 2D navigation maps** and **3DGS asset formats**.

## Data Flow

```text
Capture / Mapping
  map_recording.sh (legacy Odin workflow) or pointcloud_saver or Q9000 direct scan export  -->  .pcd / .bin map source
        |
        +-- pcd_to_pgm.py (core) ----------> .pgm + .yaml (2D navigation map)
        |       |
        |       +-- pgm_downsample.py -----> Optional downsampling for oversized PGM maps
        |       +-- Deploy to robot: pgm/yaml -> ros_ws/src/map_planner/maps/ (or ros2_ws/src/map_planner/maps/)
        |                            .bin     -> customer_file/
        |
        +-- pgm_to_irsim_world.py ---------> irsim simulation world (env.yaml + planner.yaml)
        |                                    output -> NeuPAN/example/sw01_bench/<scene>/
        |
        +-- 3DGS / Asset Conversion:
              bin_map_to_ply_* or ply_to_pcd.py (relocalization .bin / .pcd / .ply conversion & filtering)
              pcd_to_ply.py
```

## File Inventory

| Script / Binary | Purpose | Typical Usage |
|---|---|---|
| `pcd_to_pgm.py` | **Core**: Convert PCD -> PGM/YAML navigation map. Supports ascii/binary/binary_compressed (built-in pure-Python LZF, zero external deps); Z slice = `[lidar-height - slice-below, lidar-height + slice-above]`. | `python3 pcd_to_pgm.py <in.pcd> -o <output_prefix_without_ext> --lidar-height 0.6 --slice-below 0.3 --slice-above 0.2` |
| `pgm_downsample.py` | Downsample PGM using `map_server` occupancy threshold semantics (avoids intermediate gray artifacts). Backup utility for oversized maps. | `python3 pgm_downsample.py <in.pgm> --factor 2` |
| `pgm_to_irsim_world.py` | Crop PGM -> greedy rectangle merging into obstacles -> generate irsim `env.yaml` + `planner.yaml` for real-map simulation scenes. | `python3 pgm_to_irsim_world.py --pgm x.pgm --map-yaml x.yaml --crop=x,y,w,h --start=... --goal=... --out <scene_dir> [--preview]` |
| `ply_to_pcd.py` | 3DGS Gaussian PLY -> PCD (opacity/scale filtering + optional voxel downsampling for SLAM PCD comparison). | `python3 ply_to_pcd.py <in.ply> -o <out.pcd> [--min-opacity 0.1]` |
| `pcd_to_ply.py` | PLY/PCD -> GPU voxel downsampling -> PLY/GLB (3DGS asset export; requires `open3d`). | `python3 pcd_to_ply.py <in.ply> --voxel 0.1` |
| `map_recording.sh` | Legacy Odin point-cloud mapping helper (retained for Odin-only setups). | `./map_recording.sh <name> [VOXEL_SIZE] [APPLY_FILTER]` |
| `bin_map_to_ply_amd64` / `bin_map_to_ply_arm64` | Convert relocalization `.bin` map files to `.ply` (use `amd64` on x86_64, `arm64` on Jetson). | `./bin_map_to_ply_amd64 --help` |

## Notes

- Pass only the **output prefix** (without extension) to `pcd_to_pgm.py -o`; the script appends `.pgm` and `.yaml` automatically.
- Always deploy matching `.pgm`/`.yaml` and relocalization `.bin` pairs when switching maps (`--map=<name>`).
- 3DGS utilities (`ply_to_pcd.py`, `pcd_to_ply.py`, `bin_map_to_ply_*`) are optional offline conversion tools.
