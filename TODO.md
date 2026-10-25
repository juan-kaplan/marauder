# TODO — TP1 Teach & Repeat

Grading: encoders 4 pts · LiDAR 4 pts · mixed 2 pts.

The architecture, interface contract and work split live in
[`docs/tp1/arquitectura.md`](docs/tp1/arquitectura.md).

## Done

- [x] `localization_node` with `mode:=encoders|lidar|mixed`, `/rel_odom` and the `odom → reference → robot_frame` TF
- [x] Teach: keyframes by distance/angle, saved to `map_dir` (`map.json` + `scans.npz`)
- [x] Repeat: ICP against the current keyframe, setpoint in the robot frame, keyframe advance, stop at the last one
- [x] Repeat TF frames (`map_current_keyframe`, `map_next_keyframe`, `setpoint`) and `repeat.rviz`
- [x] `teach_launch.py` / `repeat_launch.py`, `sim_launch.py`, README with sim + TB4 instructions
- [x] LiDAR mount through TF, wheels selected by joint name
- [x] Humble/Jazzy support: `rota_common` replaces `tf_transformations`, `cmd_vel_type:=auto`, venv + `requirements.txt`
- [x] ROS dependencies declared in every `package.xml`
- [x] Repo restructured by function: `rota_common`, `rota_localization` (with the wheel and LiDAR odometry nodes), `rota_mapping`, `rota_control`, `rota_bringup` (launches, RViz, simulation)

## TurtleBot 4

- [x] Check the TB4 bags: `/tb4_1` namespace, `scan` reliable, ticks and `odom` best effort, `cmd_vel` is `Twist`, LiDAR under `base_link` (x -0.04, yaw +90°), `base_footprint` not linked to it
- [x] Robot profiles `robot:=tb3_sim|tb4` (`rota_bringup/config/<robot>.yaml`) and `ns:=` in the launches
- [x] Private TF: our nodes publish on `/tf`, `tb4_launch.py` adds the LiDAR mount on `/tf_static`, so the Create 3's TF stays out of our tree
- [x] Tick input (`wheel_source: ticks`) in `localization_node` and `wheel_odometry_node`
- [x] Wheel geometry measured against the Create 3 `/odom`: 4.37e-4 m per tick, separation 0.2306 m (effective r about 0.0352, about 506 ticks per revolution)
- [ ] Install `ros-humble-irobot-create-msgs` on the laptop and run the ticks on the robot
- [ ] Verify on the robot: namespace, units of `wheel_vels` (one wheel turn by hand), the Create 3 `cmd_vel` QoS and timeout, laptop and robot clock offset, undock
- [ ] Record a bag of our own with a measured straight line and a measured turn, to check `meters_per_tick` and `wheel_separation` (the course bags have no ground truth)
- [ ] First Teach and Repeat on the TB4 in `mixed` mode

## Environment

- [ ] Verify the build and a sim run on a Jazzy machine
- [ ] Make sure the laptop, robot and any Docker setup run the **same** distro
- [ ] Package metadata: maintainer and description of `rota_control`

## Deliverables

- [ ] Plot of Teach vs Repeat trajectories for each localization mode
- [ ] 5-minute presentation: Teach path plot + Teach-vs-Repeat comparison per mode
- [ ] Live demo on the TB4
