# 2D Occupancy Grid Maps (`map_planner/maps`)

Place your 2D occupancy grid maps (`<map_name>.pgm` and `<map_name>.yaml`) in this directory.

## Naming & Pairing Convention

When launching the stack with `--map=<map_name>`, three files with the **same base name** are loaded:

| File | Location | Purpose |
|---|---|---|
| `<map_name>.pgm` | `ros_ws/src/map_planner/maps/<map_name>.pgm` | 2D occupancy grid image (`map_server` / `map_planner`) |
| `<map_name>.yaml` | `ros_ws/src/map_planner/maps/<map_name>.yaml` | Map resolution and origin metadata |
| `<map_name>.bin` | `customer_file/<map_name>.bin` | Odin1 3D relocalization map |

## Generating `.pgm` and `.yaml`

- **From an Odin1 live SLAM session:** use [`scripts/map_processing/map_recording.sh`](../../../../scripts/map_processing/map_recording.sh).
- **From a `.pcd` point cloud (Q9000 / Odin1 + MindCloud Studio):** use [`scripts/map_processing/pcd_to_pgm.py`](../../../../scripts/map_processing/pcd_to_pgm.py) (see the [Map Building & Processing Guide](../../../../docs/map-tutorial/map-tutorial.md)).
- **Placeholder map:** `empty.pgm` + `empty.yaml` (100 m × 100 m free space) is provided for dry-run testing (`--map=empty`).
