# marauder

*Leaves a trail, walks it back.*

ROS 2 packages (Humble and Jazzy) built for a mobile robotics course: a set of
mobile-robot nodes (wheel and LiDAR odometry, local localization, occupancy-grid mapping,
a differential-drive controller) and the Teach-and-Repeat navigation built on them.

This repo lives inside a colcon workspace as `<workspace>/src/marauder`.
It does not include the workspace's `build/`, `install/`, or `log/`
directories (colcon regenerates those) — see [Setup](#setup) below.

## Packages

| Package | Node(s) | What it does |
|---|---|---|
| [`rota_common`](rota_common) | none | Shared library: SE(2) poses, message conversions, scan projection, ICP, the Teach and Repeat map, and a `/cmd_vel` publisher that sends `Twist` or `TwistStamped` to match the robot/simulator. |
| [`rota_localization`](rota_localization) | `localization_node`, `wheel_odometry_node`, `lidar_odometry_node` | `localization_node` is layer 1 of Teach and Repeat (`mode:=encoders\|lidar\|mixed`). The two odometry nodes are standalone: wheel odometry from `/joint_states` on `/wheel_odom`, and ICP odometry against a reference scan on `/lidar_odom`. They publish the `odom` TF only with `publish_tf:=true`. |
| [`rota_mapping`](rota_mapping) | `mapper_node` | Occupancy grid on `/map` from `/scan` and the pose in `odom_topic` (default `/localization/odom`). |
| [`rota_control`](rota_control) | `diff_control_node`, `teach_node`, `repeat_node` | Single-goal differential-drive controller, and Teach-and-Repeat: records anchor keyframes during a demonstration and replays them, closing the loop on the relative pose error with ICP against each keyframe's scan. |
| [`rota_bringup`](rota_bringup) | none | Launch files (simulation, Teach, Repeat, controller, odometry, mapper), RViz configs and, later, robot profiles. |

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
plot scripts) are listed in [`requirements.txt`](requirements.txt). Install
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

Each step gives the **TurtleBot 3 (simulation)** commands and then the **TurtleBot 4 (robot)** ones. Read [Deploying on Physical TurtleBot 4](#deploying-on-physical-turtlebot-4) first for what the robot needs (message package, namespace, clocks, undock).

### 1. Terminal 1: start the simulation (TB3) or the LiDAR mount (TB4)

From the workspace root, in every terminal:

```bash
source /opt/ros/<distro>/setup.bash
source src/marauder/.venv/bin/activate
source install/setup.bash
```

**TB3:**

```bash
# Gazebo world and a TurtleBot3 Burger (gui:=false for no window)
ros2 launch rota_bringup sim_launch.py
```

**TB4:** the robot is already running, so there is nothing to simulate. Undock it, check that `ros2 topic list` shows `/tb4_1/scan` and `/tb4_1/wheel_ticks`, and publish the LiDAR mount once:

```bash
ros2 launch rota_bringup tb4_launch.py
```

### 2. Terminal 2: Teach (demonstrate and record)

**TB3:**

```bash
# Localization (mode: encoders, lidar or mixed) and the Teach recorder
ros2 launch rota_bringup teach_launch.py mode:=mixed map_dir:=data/map rviz:=true

# Terminal 3: drive the robot along the path to teach
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**TB4:**

```bash
ros2 launch rota_bringup teach_launch.py robot:=tb4 ns:=/tb4_1 mode:=mixed map_dir:=data/map rviz:=true

# Terminal 3: drive the robot along the path to teach
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/tb4_1/cmd_vel
```

Use `mixed` on the TB4 (see below). End the Teach in free space, at least 0.2 m from any obstacle.

* **Keyframes** are created by the localizer whenever the robot moves $\ge 0.30\text{ m}$ or turns $\ge 0.30\text{ rad}$ (`tau_d`, `tau_theta`). Each one keeps its LiDAR scan and an ICP edge to the previous one.
* **Save**: press `Ctrl+C` in Terminal 2 (or call `ros2 service call /teach_node/save std_srvs/srv/Trigger`). The map goes to `map_dir`: `map.json` and `scans.npz`.

### 3. Terminal 2: Repeat (autonomous replay)

The robot has to be at the pose where the Teach started. **TB3:** restart the simulation. **TB4:** drive or carry the robot back to the start, and keep `tb4_launch.py` running. Then:

**TB3:**

```bash
ros2 launch rota_bringup repeat_launch.py mode:=mixed map_dir:=data/map rviz:=true
```

**TB4:**

```bash
ros2 launch rota_bringup repeat_launch.py robot:=tb4 ns:=/tb4_1 mode:=mixed map_dir:=data/map rviz:=true
```

* The robot will:
  1. Load the keyframes from `map_dir`.
  2. Track its pose in the active keyframe's frame: wheel/LiDAR odometry from the localizer, corrected by ICP against the keyframe's scan.
  3. Publish the next keyframe, seen from the robot, as `/repeat/setpoint`.
  4. Follow it with `diff_control_node` and advance keyframes until the last one is reached.
* The Repeat starts **paused**: the robot stays still until you start it from another terminal. The same call pauses and resumes it (the node keeps tracking while paused):

```bash
ros2 service call /repeat_node/run std_srvs/srv/SetBool "{data: true}"   # start / resume
ros2 service call /repeat_node/run std_srvs/srv/SetBool "{data: false}"  # pause
```

### Plotting a run

To compare a Repeat with its Teach, record the reference pose and the localizer's pose during each run, then plot the two bags with the Teach's map. The reference is the simulator's ground truth (`/odom`); the TB4 has none, so use the Create 3's `/tb4_1/odom` there (wheels, gyro and optical flow, not truth).

**TB3:**

```bash
ros2 bag record -o teach_bag /odom /localization/odom      # during the Teach, then Ctrl+C
ros2 bag record -o repeat_bag /odom /localization/odom     # during the Repeat, then Ctrl+C
python3 src/marauder/rota_bringup/scripts/plot_teach_vs_repeat.py teach_bag repeat_bag data/map run.png
```

**TB4:**

```bash
ros2 bag record -o teach_bag /tb4_1/odom /localization/odom
ros2 bag record -o repeat_bag /tb4_1/odom /localization/odom
python3 src/marauder/rota_bringup/scripts/plot_teach_vs_repeat.py teach_bag repeat_bag data/map run.png --ref /tb4_1/odom
```

The script needs ROS sourced (it reads the bags) and `matplotlib`. It draws the taught path, the repeated path and the repeat as the localizer saw it, with the keyframes, next to the distance from the repeated path to the taught one over time, and prints the mean, the maximum and the end error. Both runs have to start from the same pose (restart the simulation between them).

---

## RViz Visualizations & Topics

Preconfigured RViz configuration files are provided in [`rota_bringup/config/`](rota_bringup/config):

* **[`teach.rviz`](rota_bringup/config/teach.rviz)**: Displays the keyframes (`/teach/keyframes`, arrows), LiDAR scan (`/scan`), and the accumulating route (`/teach/path`).
* **[`repeat.rviz`](rota_bringup/config/repeat.rviz)**: Displays both paths simultaneously for real-time comparison:
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
# Wheel odometry (/wheel_odom) and LiDAR odometry (/lidar_odom); publish_tf:=true also publishes odom -> robot
ros2 launch rota_bringup wheel_odometry_launch.py
ros2 launch rota_bringup lidar_odometry_launch.py

# Occupancy grid (/map) from the pose in odom_topic
ros2 launch rota_bringup mapper_launch.py odom_topic:=/wheel_odom

# Standalone Single-Goal Differential-Drive Controller
ros2 launch rota_bringup diff_control_launch.py
```

---

## Deploying on Physical TurtleBot 4

For a physical **TurtleBot 4** (iRobot Create 3 base + RPLIDAR) every launch takes `robot:=tb4 ns:=/tb4_1`. The profile `rota_bringup/config/tb4.yaml` sets the frame, the wheel geometry, `use_sim_time: false`, the scan range and `cmd_vel_type: twist`; `ns` is the robot's namespace (`tb4_1` on the course bags, check it with `ros2 topic list`). The default is `robot:=tb3_sim`, the simulation. The numbers come from the course bags recorded on the TB4, compared with the Create 3's own `/odom`.

### What is different from the simulation

1. **Ticks, not `/joint_states`.** The TB4 publishes no `/joint_states`. With `wheel_source: ticks` the wheel readers (`localization_node`, `wheel_odometry_node`) read `<ns>/wheel_ticks` (`irobot_create_msgs/msg/WheelTicks`) and use `meters_per_tick` (4.37e-4, about 1.5% between bags) and `wheel_separation` (0.2306 m). This needs the message package on the laptop:
   ```bash
   sudo apt install ros-humble-irobot-create-msgs
   ```
2. **Private TF.** The Create 3 publishes `odom -> base_link` and `odom -> base_footprint` on `<ns>/tf`. Our nodes publish on `/tf` and never read the robot's TF, so the two trees do not meet and nothing has to be turned off on the robot. The LiDAR mount is not in our TF, so `tb4_launch.py` publishes it on `/tf_static` (`base_link -> rplidar_link`: x -0.04, z 0.1929, yaw +90 degrees, from the robot's own `tf_static`). `robot_frame` is `base_link`: `base_footprint` has no connection to the LiDAR in the robot's tree.
3. **Scan.** 720 beams at about 8 Hz, range 0.15 to 12 m. The TB3 limit of 3.5 m kept only 30 to 45% of the beams in the lab, so the profile uses `max_scan_range: 8.0` (replayed on the bags, the lidar-only drift fell from 2.2 m to 0.5 m on a 24 m lap, 12 m was no better). The RPLIDAR publishes `scan` reliable; our subscribers use best effort, which connects to both.
4. **Time.** There is no `/clock`: the profile sets `use_sim_time: false`. Keep the laptop and the robot clocks in sync (chrony or NTP): the Repeat stops if the newest TF is more than 0.5 s older than the laptop clock, and our TF carries the robot's stamps.
5. **Velocity commands.** The Create 3 takes `geometry_msgs/Twist` on `<ns>/cmd_vel`. `diff_control_node` is started with `cmd_vel_type: twist` by the profile (`auto` and `twist_stamped` still exist; on a simulator bridge that expects `TwistStamped` use them). For teleoperation during the Teach use `teleop_twist_keyboard` (remap `cmd_vel` to `<ns>/cmd_vel`) or the Bluetooth controller.
6. **DDS.** The laptop and the robot need the same `ROS_DOMAIN_ID`; `ros2 topic list` must show `<ns>/scan` and `<ns>/wheel_ticks`.
7. **Undock before driving:**
   ```bash
   ros2 action send_goal <ns>/undock irobot_create_msgs/action/Undock {}
   ```

### Running Teach and Repeat on the TurtleBot 4

The commands are in [Teach and Repeat](#teach-and-repeat-tnr), steps 1 to 3 (the TB4 blocks). The Repeat starts paused there too, and the same `/repeat_node/run` call starts it.

`wheel_odometry_launch.py`, `lidar_odometry_launch.py` and `mapper_launch.py` take the same `robot:=` and `ns:=`. Use `mixed` on the TB4: on the bags the LiDAR alone drifted much more than the ticks, and lost track across a multi-second scan gap.

## License

Apache License 2.0 — see [LICENSE](LICENSE).
