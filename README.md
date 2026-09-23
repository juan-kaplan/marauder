# marauder

*Leaves a trail, walks it back.*

ROS 2 (Jazzy) packages built for a mobile robotics course, covering wheel
odometry, LiDAR occupancy-grid mapping, ICP scan-matching odometry, and
Teach-and-Repeat differential-drive autonomous navigation.

This repo lives inside a colcon workspace as `<workspace>/src/marauder`.
It does not include the workspace's `build/`, `install/`, or `log/`
directories (colcon regenerates those) — see [Setup](#setup) below.

## Packages

| Package | Node(s) | What it does |
|---|---|---|
| [`rota_robmob`](rota_robmob) | `odometry`, `mapper` | Differential-drive wheel odometry (SE(2) pose from encoder/joint data, published as `nav_msgs/Odometry`) and LiDAR-based occupancy-grid mapping (`nav_msgs/OccupancyGrid`) from `sensor_msgs/LaserScan`. |
| [`rota_icp`](rota_icp) | `lidar_odometry` | LiDAR odometry via point-to-point ICP (Open3D), aligning consecutive 2D scans to estimate relative motion. |
| [`rota_control`](rota_control) | `diff_control_node`, `teach_node`, `repeat_node` | Teach-and-Repeat navigation system: records anchor keyframes during demonstration and autonomously repeats the trajectory by closing the loop on relative pose error via Open3D ICP scan matching and differential-drive kinematic control. |

Course material and assignment briefs for each package live under
[`docs/`](docs) (`docs/clase2`, `docs/clase3`, `docs/clase4`), including the
reference papers behind Teach-and-Repeat and Lie-group theory
([Sprunk et al. 2013](docs/clase4/Sprunk-2013-Lidar%20based%20teach%20and%20repeat.pdf), [Solà 2019](docs/clase4/Sola-2019-Micro%20Lie.pdf), and [diff.md](docs/clase4/control/diff.md)).

## Setup

Clone this repo into the `src/` directory of your colcon workspace:

```bash
cd ~/Documents/robotica_ws/src
git clone <this-repo-url> marauder
cd ..
```

### Python Virtual Environment (`uv`)

Create a virtual environment with system site packages enabled (so ROS 2 Jazzy bindings like `rclpy` are accessible) and install `open3d`:

```bash
# Using uv (recommended)
uv venv --python /usr/bin/python3 --system-site-packages .venv
source .venv/bin/activate
uv pip install open3d
```

### Build & Run Tests

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

colcon build --symlink-install
source install/setup.bash

# Run unit tests
python3 -m pytest src/marauder/rota_control/test/test_tnr.py -v
```

---

## Teach and Repeat (TnR)

The Teach-and-Repeat pipeline operates entirely without a global metric SLAM map or global path planning (A*, Dijkstra). It represents the trajectory as a sequence of **anchor keyframes** $a_k = (l_k, x_k)$ and relative transformations $T_{(k, k+1)} = x_{k+1} \ominus x_k \in SE(2)$, closing the loop on relative pose error using live LiDAR scan matching (Open3D ICP).

### 1. Terminal 1 — Start the Simulation

From the workspace root (`~/Documents/robotica_ws`):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launches Gazebo world, TurtleBot3 Burger, RViz, and teleop keyboard
ros2 launch rota_robmob turtlebot3_simulation_environment_launch.py
```

### 2. Terminal 2 — Teach Phase (Demonstrate & Record)

From the workspace root (`~/Documents/robotica_ws`):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source .venv/bin/activate

# Launch the Teach recorder (optionally pass rviz:=true to view teach.rviz)
ros2 launch rota_control teach_launch.py rviz:=true
```

* **Drive the robot**: Use the teleop keyboard window (`w`/`a`/`s`/`d`/`x`) to drive along your desired path.
* **Anchor sampling**: Keyframes are automatically inserted whenever the robot moves $\ge 0.30\text{ m}$ or turns $\ge 17^\circ$.
* **Stop recording**: Press `Ctrl+C` in Terminal 2 (or call `/teach_node/stop_recording`). The trajectory is saved to `data/trajectory.json` and `data/trajectory_scans.npz`.

### 3. Terminal 2 — Repeat Phase (Autonomous Replay & Tracking)

From the workspace root (`~/Documents/robotica_ws`):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source .venv/bin/activate

# Launch autonomous trajectory tracking with repeat.rviz
ros2 launch rota_control repeat_launch.py rviz:=true
```

* The robot will:
  1. Load anchors from `data/trajectory.json` and point clouds from `data/trajectory_scans.npz`.
  2. Perform Open3D ICP scan matching against reference anchor scans, seeded with wheel odometry displacement.
  3. Compute relative target setpoints in the robot frame ($T_{target, robot} = T_{k, k+1} \ominus s_k(t)$).
  4. Follow the path using the Siegwart polar kinematic controller and advance anchors automatically until the final goal is reached.

---

## RViz Visualizations & Topics

Preconfigured RViz configuration files are provided in [`rota_control/config/`](rota_control/config):

* **[`teach.rviz`](rota_control/config/teach.rviz)**: Displays the demonstrated anchor points (arrows), LiDAR scan (`/scan`), and the accumulating route (`/teach/path`).
* **[`repeat.rviz`](rota_control/config/repeat.rviz)**: Displays both paths simultaneously for real-time comparison:
  * **Taught Reference Path**: `/repeat/taught_path` (Bright Green, width 0.04m)
  * **Actual Repeat Path**: `/repeat/actual_path` (Cyan / Blue, width 0.035m)
  * **Status & Target Markers**: `/repeat/markers` (shows active waypoint arrow and live status)
  * **Dynamic TF Frames**:
    * `current_anchor`: Coordinate frame of the currently active keyframe anchor.
    * `target_anchor`: Coordinate frame of the next goal anchor.
    * `base_link_icp`: Measured robot base pose according to LiDAR ICP scan matching.

> **Tip**: In RViz, set **Fixed Frame** to `current_anchor` to observe the robot's motion relative to the active keyframe reference.

---

## Additional Launch Files

```bash
# Lidar Odometry via ICP
ros2 launch rota_icp odometry_launch.py

# Standalone Single-Goal Differential-Drive Controller
ros2 launch rota_control diff_control_launch.py
```

*(Note: If running without Gazebo, e.g. replaying a rosbag that only has `/joint_states`, append `start_odometry:=true` to `teach_launch.py` or `repeat_launch.py` to start `rota_robmob`'s wheel odometry node automatically).*

---

## Deploying on Physical TurtleBot 4

When transitioning from Gazebo simulation to a physical **TurtleBot 4** (iRobot Create 3 base + RPLIDAR):

### Key Differences & What Changed

1. **Simulation Time (`use_sim_time:=false`)**:
   * On physical hardware there is no `/clock` topic. **You must set `use_sim_time:=false`**; otherwise, ROS timers will freeze.
2. **LiDAR QoS Profile**:
   * The physical RPLIDAR driver publishes `sensor_msgs/msg/LaserScan` with `BEST_EFFORT` reliability (`qos_profile_sensor_data`). Both `teach_node` and `repeat_node` now use `qos_profile_sensor_data`, ensuring seamless compatibility with both physical hardware and simulation.
3. **Base Odometry (`start_odometry:=false`)**:
   * The Create 3 base automatically computes fused wheel odometry and broadcasts `odom -> base_link` on `/odom`. Keep `start_odometry:=false` (default).
4. **Velocity Commands (`/cmd_vel`)**:
   * TurtleBot 4 in ROS 2 Jazzy accepts `geometry_msgs/msg/TwistStamped` on `/cmd_vel` (`use_stamped_vel:=true`, default).
   * For keyboard teleoperation during the Teach phase, launch `teleop_twist_keyboard` with stamped mode:
     ```bash
     ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
     ```
     *(Or simply drive using the TurtleBot 4 Bluetooth joy controller).*
5. **DDS Network Discovery**:
   * Ensure your workstation and TurtleBot 4 share the same `ROS_DOMAIN_ID`:
     ```bash
     export ROS_DOMAIN_ID=0  # or your lab's assigned domain ID
     ros2 topic list         # verify /scan and /odom are received
     ```
6. **Undock Before Driving**:
   * If the robot is on the home dock, undock before running navigation:
     ```bash
     ros2 action send_goal /undock irobot_create_msgs/action/Undock {}
     ```

### Commands for TurtleBot 4

**1. Teach Phase on TurtleBot 4:**
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source .venv/bin/activate

# Launch Teach node (with use_sim_time:=false)
ros2 launch rota_control teach_launch.py use_sim_time:=false rviz:=true
```
Drive the robot manually along the path. Press `Ctrl+C` when done.

**2. Repeat Phase on TurtleBot 4:**
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source .venv/bin/activate

# Launch Repeat node (with use_sim_time:=false)
ros2 launch rota_control repeat_launch.py use_sim_time:=false rviz:=true
```

## License

Apache License 2.0 — see [LICENSE](LICENSE).
