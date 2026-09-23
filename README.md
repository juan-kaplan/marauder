# marauder

*Leaves a trail, walks it back.*

ROS2 (Jazzy) packages built for a mobile robotics course, covering wheel
odometry, LiDAR occupancy-grid mapping, ICP scan-matching odometry, and
Teach-and-Repeat differential-drive control.

This repo lives inside a colcon workspace as `<workspace>/src/marauder`.
It does not include the workspace's `build/`, `install/`, or `log/`
directories (colcon regenerates those) — see [Setup](#setup) below.

## Packages

| Package | Node(s) | What it does |
|---|---|---|
| [`rota_robmob`](rota_robmob) | `odometry`, `mapper` | Differential-drive wheel odometry (SE(2) pose from encoder/joint data, published as `nav_msgs/Odometry`) and LiDAR-based occupancy-grid mapping (`nav_msgs/OccupancyGrid`) from `sensor_msgs/LaserScan`. |
| [`rota_icp`](rota_icp) | `lidar_odometry` | LiDAR odometry via point-to-point ICP (Open3D), aligning consecutive 2D scans to estimate relative motion. |
| [`rota_control`](rota_control) | `diff_control_node` | Differential-drive control for Teach-and-Repeat navigation: replays a taught sequence of keyframes by closing the loop on relative pose error. |

Course material and assignment briefs for each package live under
[`docs/`](docs) (`docs/clase2`, `docs/clase3`, `docs/clase4`), including the
two reference papers behind the Teach-and-Repeat and Lie-group material
(`docs/clase4/*.pdf`).

## Setup

Clone this repo into the `src/` of a colcon workspace, e.g.:

```bash
mkdir -p ~/robotica_ws/src
cd ~/robotica_ws/src
git clone <this-repo-url> marauder
cd ..
colcon build --symlink-install
source install/setup.bash
```

Requires ROS2 Jazzy. `rota_icp` additionally needs Open3D:

```bash
pip install open3d
```

### Sample data

Bag files and recorded rosbags used for offline testing/experiments are not
tracked in this repo (see `.gitignore`) — keep them in a local `data/`
directory alongside `src/`, `build/`, etc. in the workspace.

## Running

Each package ships its own `launch/` files, e.g.:

```bash
ros2 launch rota_robmob turtlebot3_simulation_environment_launch.py
ros2 launch rota_icp odometry_launch.py
ros2 launch rota_control diff_control_launch.py
```

See each package's `launch/` directory and `docs/<clase>/enunciado.md` for
parameters and expected topics.

## Known issues / TODO

This repo was consolidated from three separate per-class workspaces as-is,
carrying over a few rough edges to be cleaned up:

- `rota_robmob/setup.py` declares a `goal_pose` console-script entry point
  (`rota_robmob.goal_pose_node:main`) but `goal_pose_node.py` no longer
  exists in source — the package won't build cleanly until this is removed
  or the node is restored.
- All three `package.xml`/`setup.py` files still carry the original
  template's maintainer info (`seb` / `sebabedin@gmail.com`) and a
  placeholder `TODO: Package description`.
- Per-package `LICENSE` files (Apache-2.0, unfilled `[yyyy] [name]`
  copyright template) are duplicated across all three packages instead of
  referencing the root `LICENSE`.

## License

Apache License 2.0 — see [LICENSE](LICENSE).
