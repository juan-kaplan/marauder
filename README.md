# marauder

*Leaves a trail, walks it back.*

ROS 2 packages (Humble and Jazzy) built for a mobile robotics course, covering wheel
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
| [`rota_common`](rota_common) | — | Shared helpers: pure-NumPy replacements for `tf_transformations`, and a `/cmd_vel` publisher that sends `Twist` or `TwistStamped` to match the robot/simulator. |
| [`rota_control`](rota_control) | `diff_control_node`, `teach_node`, `repeat_node` | Teach-and-Repeat navigation system: records anchor keyframes during demonstration and autonomously repeats the trajectory by closing the loop on relative pose error via Open3D ICP scan matching and differential-drive kinematic control. |

Course material and assignment briefs for each package live under
[`docs/`](docs) (`docs/clase2`, `docs/clase3`, `docs/clase4`), including the
reference papers behind Teach-and-Repeat and Lie-group theory
([Sprunk et al. 2013](docs/clase4/Sprunk-2013-Lidar%20based%20teach%20and%20repeat.pdf), [Solà 2019](docs/clase4/Sola-2019-Micro%20Lie.pdf), and [diff.md](docs/clase4/control/diff.md)).

## Setup

Works on **ROS 2 Humble** (Ubuntu 22.04, Python 3.10) and **ROS 2 Jazzy**
(Ubuntu 24.04, Python 3.12). In the commands below, replace `<distro>` with
`humble` or `jazzy`.

> The laptop and the robot must run the **same** distro: Humble and Jazzy do not
> reliably talk to each other over DDS.

Clone this repo into the `src/` directory of your colcon workspace:

```bash
cd ~/Documents/robotica_ws/src
git clone <this-repo-url> marauder
cd ..
```

### ROS dependencies (rosdep)

```bash
source /opt/ros/<distro>/setup.bash
rosdep install --from-paths src --ignore-src -y
```

For simulation, also install the TurtleBot3 packages (Gazebo Classic on Humble,
Gazebo/gz on Jazzy):

```bash
sudo apt install ros-<distro>-turtlebot3-gazebo ros-<distro>-turtlebot3-teleop
```

### Python dependencies (venv)

Python packages that ROS doesn't provide (`open3d`, plus `matplotlib` for the
example script) are listed in [`requirements.txt`](requirements.txt). Install
them in a venv at `src/marauder/.venv` that is built on the system Python and can
see the ROS packages (`--system-site-packages`). `numpy` is pinned to `<2` to
match what the ROS packages were built against.

From the workspace root:

```bash
source /opt/ros/<distro>/setup.bash
python3 -m venv --system-site-packages src/marauder/.venv
source src/marauder/.venv/bin/activate
pip install -r src/marauder/requirements.txt
touch src/marauder/.venv/COLCON_IGNORE   # keep colcon out of the venv
```

(On Ubuntu, `python3 -m venv` needs `sudo apt install python3-venv`.)

### Build

From the workspace root, **with the venv active**:

```bash
source /opt/ros/<distro>/setup.bash
source src/marauder/.venv/bin/activate

python -m colcon build --symlink-install
source install/setup.bash
```

Use `python -m colcon` (not plain `colcon`): it makes the installed nodes start
with `#!/usr/bin/env python3`, so they run in the venv and find `open3d`.
**Activate the venv in every terminal where you run the nodes.**

---

## Teach and Repeat (TnR)

The Teach-and-Repeat pipeline operates entirely without a global metric SLAM map or global path planning (A*, Dijkstra). It represents the trajectory as a sequence of **anchor keyframes** $a_k = (l_k, x_k)$ and relative transformations $T_{(k, k+1)} = x_{k+1} \ominus x_k \in SE(2)$, closing the loop on relative pose error using live LiDAR scan matching (Open3D ICP).

### 1. Terminal 1: start the simulation

From the workspace root:

```bash
source /opt/ros/<distro>/setup.bash
source src/marauder/.venv/bin/activate
source install/setup.bash

# Gazebo world and a TurtleBot3 Burger (gui:=false for no window)
ros2 launch rota_common sim_launch.py
```

### 2. Terminal 2: Teach (demonstrate and record)

```bash
# Localization (mode: encoders, lidar or mixed) and the Teach recorder
ros2 launch rota_control teach_launch.py mode:=mixed map_dir:=data/map rviz:=true

# Terminal 3: drive the robot along the path to teach
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

* **Keyframes** are created by the localizer whenever the robot moves $\ge 0.30\text{ m}$ or turns $\ge 0.30\text{ rad}$ (`tau_d`, `tau_theta`). Each one keeps its LiDAR scan and an ICP edge to the previous one.
* **Save**: press `Ctrl+C` in Terminal 2 (or call `ros2 service call /teach_node/save std_srvs/srv/Trigger`). The map goes to `map_dir`: `map.json` and `scans.npz`.

### 3. Terminal 2: Repeat (autonomous replay)

Restart the simulation, so the robot is at the pose where the Teach started, and then:

```bash
ros2 launch rota_control repeat_launch.py mode:=mixed map_dir:=data/map rviz:=true
```

* The robot will:
  1. Load the keyframes from `map_dir`.
  2. Track its pose in the active keyframe's frame: wheel/LiDAR odometry from the localizer, corrected by ICP against the keyframe's scan.
  3. Publish the next keyframe, seen from the robot, as `/repeat/setpoint`.
  4. Follow it with `diff_control_node` and advance keyframes until the last one is reached.
* Pass `use_map_icp:=false` to repeat on the localizer alone.

---

## RViz Visualizations & Topics

Preconfigured RViz configuration files are provided in [`rota_control/config/`](rota_control/config):

* **[`teach.rviz`](rota_control/config/teach.rviz)**: Displays the keyframes (`/teach/keyframes`, arrows), LiDAR scan (`/scan`), and the accumulating route (`/teach/path`).
* **[`repeat.rviz`](rota_control/config/repeat.rviz)**: Displays both paths simultaneously for real-time comparison:
  * **Taught Reference Path**: `/repeat/taught_path` (Bright Green, width 0.04m)
  * **Actual Repeat Path**: `/repeat/actual_path` (Cyan / Blue, width 0.035m)
  * **Dynamic TF Frames**:
    * `map_current_keyframe`: Coordinate frame of the keyframe being followed.
    * `map_next_keyframe`: Coordinate frame of the next keyframe.
    * `setpoint`: The setpoint sent to the controller.

> **Tip**: In RViz, set **Fixed Frame** to `map_current_keyframe` to observe the robot's motion relative to the active keyframe.

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
   * `repeat_node` and `diff_control_node` take `cmd_vel_type:=auto|twist|twist_stamped` (default `auto`). In `auto` they look at what is subscribed to `/cmd_vel` (the robot base or the simulator bridge) and send that type; until something subscribes, nothing is sent.
   * Typically: Humble Gazebo Classic and the Create 3 base expect `Twist`; the Jazzy Gazebo (gz) bridge expects `TwistStamped`. Check with `ros2 topic info /cmd_vel -v` and force a type if `auto` picks the wrong one.
   * For keyboard teleoperation during the Teach phase, match the same type (`-p stamped:=true` for `TwistStamped`):
     ```bash
     ros2 run teleop_twist_keyboard teleop_twist_keyboard
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
source /opt/ros/<distro>/setup.bash
source src/marauder/.venv/bin/activate
source install/setup.bash

# Launch Teach node (with use_sim_time:=false)
ros2 launch rota_control teach_launch.py use_sim_time:=false rviz:=true
```
Drive the robot manually along the path. Press `Ctrl+C` when done.

**2. Repeat Phase on TurtleBot 4:**
```bash
source /opt/ros/<distro>/setup.bash
source src/marauder/.venv/bin/activate
source install/setup.bash

# Launch Repeat node (with use_sim_time:=false)
ros2 launch rota_control repeat_launch.py use_sim_time:=false rviz:=true
```

## License

Apache License 2.0 — see [LICENSE](LICENSE).
