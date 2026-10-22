# TODO — TP1 Teach & Repeat

Status review against `TP1.pdf` (Teach and Repeat on a differential-drive robot).
Grading: encoders 4 pts · LiDAR 4 pts · mixed 2 pts.

> The agreed architecture, interface contract and work split now live in
> [`docs/tp1/arquitectura.md`](docs/tp1/arquitectura.md). Where section 1 below
> disagrees with it (e.g. a single unified localization), the doc wins.

## Already done

- [x] Teach: keyframe sampling by distance/angle (`tau_d`, `tau_theta`), saved to `trajectory.json` + `_scans.npz`
- [x] Repeat: loads trajectory, ICP against the anchor scan (odom guess), Siegwart polar controller, keyframe advance, goal stop
- [x] `teach_launch.py` / `repeat_launch.py`, `teach.rviz` / `repeat.rviz`
- [x] Wheel odometry node (`rota_robmob`) and ICP LiDAR odometry (`rota_icp`)
- [x] README with sim + TB4 instructions

## 1. Spec compliance (affects sim too)

- [ ] **Unified localization node** with `mode:=encoders|lidar|mixed`, used by both Teach and Repeat
  - [ ] `encoders`: wheel odometry only
  - [ ] `lidar`: ICP scan matching only
  - [ ] `mixed`: encoder odometry while no LiDAR data arrives (per spec)
- [ ] Publish `/rel_odom` (`nav_msgs/Odometry`): robot pose relative to the last keyframe
- [ ] Teach TF: `odom → reference` and `reference → base_link`
- [ ] Repeat TF: keep `odom → reference`, `reference → base_link`, and add:
  - [ ] `base_link → map_current_keyframe`
  - [ ] `map_current_keyframe → map_next_keyframe`
  - [ ] `base_link → setpoint`
- [ ] Rename the current Repeat frames (`current_anchor`, `target_anchor`, `base_link_icp`) to the spec names; update `repeat.rviz`
- [ ] Stop `rota_icp` from publishing `/odom` + `odom → base_link` (clashes with Gazebo/Create 3 odom), or fold it into the unified node
- [ ] Stop `rota_robmob` odometry from publishing `/odom` + `odom → base_footprint` for the same reason (use its own topic/frame)

## 2. Real robot (TurtleBot 4)

- [ ] **LiDAR mounting**: transform scans `lidar → base_link` (TF lookup, as `rota_icp` already does) in Teach before saving and in Repeat before ICP. The TB4 RPLIDAR is believed to be rotated about 90°; confirm from `/tf_static`
- [ ] **Repeat independent of the odom origin**: track the pose relative to the anchor from the last good ICP and add only the odom change since then, instead of `current_odom ⊖ anchor_teach_odom` (breaks when TB4 odom isn't reset between Teach and Repeat)
- [ ] **Wheel encoders on TB4**: decide between `/joint_states` (wheel angles) and `/wheel_ticks` (`irobot_create_msgs`, about 508.8 ticks/rev; verify)
  - [ ] Select wheels by joint **name**, not index
  - [ ] Correct wheel geometry (TB3 burger ≈ r 0.033 / sep 0.160; TB4 ≈ r 0.036 / sep 0.233; verify)
- [ ] **Robot profile** (launch arg or params YAML: `robot:=tb3_sim|tb4`): topics/namespace, frames, wheel geometry, `cmd_vel_type`, `use_sim_time`
- [ ] Default `use_sim_time:=false` for the TB4 profile (nodes that wait for a valid clock hang otherwise)
- [ ] Verify against the TB4 bags:
  - [ ] Topic names / namespace
  - [ ] QoS of `/odom` and `/scan`
  - [ ] `/cmd_vel` type (`Twist` vs `TwistStamped`); `cmd_vel_type:=auto` now detects it, but confirm
  - [ ] `/tf_static` (LiDAR pose, `base_link` vs `base_footprint`)
- [ ] (Minor) Teach: time-sync scan and pose when recording a keyframe

## 3. Environment / cleanup

- [x] Humble/Jazzy support: `rota_common` replaces `tf_transformations`, `cmd_vel_type:=auto`, venv + `requirements.txt` (verified on Humble)
- [ ] Verify the build, tests and a sim run on a Jazzy machine
- [ ] Make sure the laptop, robot and any Docker setup run the **same** distro
- [x] Remove the hardcoded `.venv` path hack in `repeat_node.py` (wrong workspace path)
- [x] Guard against `o3d is None` in Repeat: Open3D is now a hard import (fails at startup with a clear error)
- [ ] Trajectory paths are relative to cwd (`data/trajectory.json`); make them explicit or absolute
- [x] Declare ROS dependencies in every `package.xml` (rosdep works on both distros)
- [ ] Package metadata: maintainer, descriptions, duplicated per-package LICENSE

## 4. Deliverables

- [ ] Script to record and plot Teach vs Repeat trajectories for each localization mode
- [ ] Simulation demo covering all of section 2
- [ ] 5-minute presentation: Teach path plot + Teach-vs-Repeat comparison per mode
- [ ] README: build/config/use for sim **and** real robot, including mode selection
- [ ] Live demo on the TB4
