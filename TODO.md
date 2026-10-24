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

## TurtleBot 4

- [ ] Turn off the Create 3's own `odom → base_link` TF (see the README)
- [ ] Robot profile (`robot:=tb3_sim|tb4`): topics, frames, wheel geometry, `cmd_vel_type`, `use_sim_time:=false`
  - [ ] Wheel geometry (TB3 burger ≈ r 0.033 / sep 0.160; TB4 ≈ r 0.036 / sep 0.233; verify)
- [ ] Verify against the TB4 bags:
  - [ ] Topic names / namespace
  - [ ] QoS of `/odom` and `/scan`
  - [ ] `/cmd_vel` type (`Twist` vs `TwistStamped`)
  - [ ] `/tf_static` (LiDAR pose, `base_link` vs `base_footprint`)

## Environment

- [ ] Verify the build and a sim run on a Jazzy machine
- [ ] Make sure the laptop, robot and any Docker setup run the **same** distro
- [ ] Package metadata: maintainer, descriptions, duplicated per-package LICENSE

## Deliverables

- [ ] Plot of Teach vs Repeat trajectories for each localization mode
- [ ] 5-minute presentation: Teach path plot + Teach-vs-Repeat comparison per mode
- [ ] Live demo on the TB4
