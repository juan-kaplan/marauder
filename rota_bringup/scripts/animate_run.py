#!/usr/bin/env python3
"""
Animate a run from its bag, or draw the map a Teach built.

  python3 animate_run.py run bag out.gif [--map map_dir] [--ref /odom] [--scan /scan]
                         [--setpoint /repeat/setpoint] [--ns /tb4_1] [--still out.png] [--es]
  python3 animate_run.py map map_dir out.png

run: top-down, in the frame of the first reference pose. The robot is an arrow
moving along the reference odom, with its LiDAR scan around it (older scans stay
faint, so the walls build up). With --map the taught path and keyframes are drawn
in grey; with a /repeat/setpoint in the bag, the target is a dot and the stored
scan of that keyframe is drawn on it. --still also saves the last frame (with every
scan accumulated) as a PNG; --es labels in Spanish. out.gif or out.mp4 picks the writer.
The bag needs the reference odom, the scan and /tf_static (the LiDAR mount);
without a mount in the bag pass --mount x y yaw. Run it with ROS sourced.
"""

import argparse
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import rosbag2_py

from plot_teach_vs_repeat import from_frame, read_odom, to_frame
from rota_common import se2, tnr_map


def read_topic(bag, suffix, msg_type):
    """Messages of the first topic that ends with suffix, as (name, [(t, msg)])."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('', ''))
    out, name = [], None
    while reader.has_next():
        n, data, t = reader.read_next()
        if n.endswith(suffix) and name in (None, n):
            name = n
            out.append((t * 1e-9, deserialize_message(data, msg_type)))
    return name, out


def yaw_of(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def lidar_mount(bag, child, frame):
    """Pose of the scan frame seen from the odom child frame, from /tf_static."""
    _, msgs = read_topic(bag, 'tf_static', TFMessage)
    parent_of = {}
    for _, m in msgs:
        for t in m.transforms:
            tr = t.transform.translation
            parent_of[t.child_frame_id.lstrip('/')] = (t.header.frame_id.lstrip('/'), (tr.x, tr.y, yaw_of(t.transform.rotation)))
    pose, f = (0.0, 0.0, 0.0), frame.lstrip('/')
    child = child.lstrip('/')
    while f != child:
        if f not in parent_of:
            return None
        f, step = parent_of[f]
        pose = se2.compose(step, pose)
    return pose


def scan_points(msg, max_range):
    r = np.array(msg.ranges)
    a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
    ok = np.isfinite(r) & (r >= msg.range_min) & (r <= min(msg.range_max, max_range))
    return np.c_[r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]


def place(pose, pts):
    """Points given in a robot frame, seen in the frame the pose is in."""
    x, y = from_frame(pose, pts[:, 0], pts[:, 1])
    return np.c_[x, y]


def draw_map(ax, map_dir):
    _, kfs = tnr_map.load(map_dir)
    poses = np.array(tnr_map.chain_poses(kfs))
    for kf, p in zip(kfs, poses):
        ax.plot(*place(p, kf.points).T, '.', color='0.75', ms=1)
    ax.plot(poses[:, 0], poses[:, 1], '-', color='0.35', lw=2)
    ax.plot(poses[:, 0], poses[:, 1], 'ko', ms=4)
    ax.quiver(poses[:, 0], poses[:, 1], np.cos(poses[:, 2]), np.sin(poses[:, 2]), scale=25, width=0.003)
    return kfs, poses


def map_figure(a):
    fig, ax = plt.subplots(figsize=(7, 6))
    kfs, poses = draw_map(ax, a.map_dir)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f'map built by the Teach: {len(kfs)} keyframes, {sum(len(k.points) for k in kfs)} points')
    fig.tight_layout()
    fig.savefig(a.out, dpi=130)


def run_animation(a):
    ns = a.ns.rstrip('/')
    rt, rx, ry, ryaw = read_odom(a.bag, ns + a.ref)
    _, scans = read_topic(a.bag, a.scan, LaserScan)
    _, sets = read_topic(a.bag, a.setpoint, PoseStamped)
    if not scans:
        raise SystemExit(f'no {a.scan} in {a.bag}')
    child = read_child_frame(a.bag, ns + a.ref)
    mount = tuple(a.mount) if a.mount else lidar_mount(a.bag, child, scans[0][1].header.frame_id)
    if mount is None:
        raise SystemExit(f'no {scans[0][1].header.frame_id} -> {child} in /tf_static: pass --mount x y yaw')

    origin = (rx[0], ry[0], ryaw[0])
    rx, ry = to_frame(origin, rx, ry)
    ryaw = ryaw - origin[2]
    st = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in scans])
    sp = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in sets])

    def pose_at(t):
        return (np.interp(t, rt, rx), np.interp(t, rt, ry), np.interp(t, rt, np.unwrap(ryaw)))

    fig, ax = plt.subplots(figsize=(7, 6))
    kfs = poses = None
    if a.map:
        kfs, poses = draw_map(ax, a.map)
        for art in list(ax.collections) + list(ax.lines):
            art.set_alpha(0.5)
    ax.plot(rx, ry, '-', color='tab:blue', lw=0.6, alpha=0.4)
    old = ax.scatter([], [], s=1, c='0.6')
    live = ax.scatter([], [], s=3, c='tab:red')
    stored = ax.scatter([], [], s=3, c='tab:green')
    target, = ax.plot([], [], 'o', color='tab:green', ms=9)
    arrow, = ax.plot([], [], 'b-', lw=2.5)
    trail, = ax.plot([], [], '-', color='tab:blue', lw=1.5)
    clock = ax.text(0.02, 0.97, '', transform=ax.transAxes, va='top')
    if sets:
        ax.legend([live, stored, target], (['scan actual', 'scan guardado del objetivo', 'objetivo'] if a.es else ['live scan', 'stored scan of the target', 'target']), loc='lower right', fontsize=8)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(a.title or os.path.basename(a.bag.rstrip('/')))

    # fixed view: the path plus most of the scan points
    allp = [place(pose_at(st[i]), place(mount, scan_points(scans[i][1], a.max_range)))
            for i in range(0, len(scans), max(1, len(scans) // 40))]
    allp = np.vstack(allp + [np.c_[rx, ry]])
    lo, hi = np.percentile(allp, 1, axis=0) - 0.5, np.percentile(allp, 99, axis=0) + 0.5
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])

    t0, t1 = rt[0], rt[-1]
    n = int(a.seconds * a.fps)
    times = np.linspace(t0, t1, n)
    faint, state = [], {'k': 0}

    def frame(i):
        t = times[i]
        pose = pose_at(t)
        j = max(np.searchsorted(st, t, side='right') - 1, 0)
        pts = place(pose_at(st[j]), place(mount, scan_points(scans[j][1], a.max_range)))
        if i % 2 == 0:
            faint.append(pts[::3])
            old.set_offsets(np.vstack(faint))
        live.set_offsets(pts)
        past = rt <= t
        trail.set_data(rx[past], ry[past])
        arrow.set_data([pose[0], pose[0] + 0.25 * np.cos(pose[2])], [pose[1], pose[1] + 0.25 * np.sin(pose[2])])
        if len(sets):
            k = max(np.searchsorted(sp, t, side='right') - 1, 0)
            m = sets[k][1].pose
            rel = (m.position.x, m.position.y, yaw_of(m.orientation))
            goal = se2.compose(pose, rel)
            target.set_data([goal[0]], [goal[1]])
            if kfs:
                near = [state['k'] + d for d in range(4) if state['k'] + d < len(kfs)]
                cost = [np.hypot(*(np.array(poses[q][:2]) - goal[:2])) + 0.3 * abs(se2.normalize_angle(poses[q][2] - goal[2]))
                        for q in near]
                state['k'] = near[int(np.argmin(cost))]
                stored.set_offsets(place(goal, kfs[state['k']].points))
        clock.set_text(f't = {t - t0:5.1f} s')

    if a.still:
        for i in range(n):
            frame(i)
        fig.savefig(a.still, dpi=130)
        faint.clear()

    anim = FuncAnimation(fig, frame, frames=n)
    writer = FFMpegWriter(fps=a.fps) if a.out.endswith('.mp4') else PillowWriter(fps=a.fps)
    anim.save(a.out, writer=writer, dpi=a.dpi)
    print(f'{a.out}: {n} frames, {(t1 - t0):.0f} s of data in {a.seconds} s, {os.path.getsize(a.out) / 1e6:.1f} MB')


def read_child_frame(bag, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('', ''))
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name == topic:
            return deserialize_message(data, Odometry).child_frame_id


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('bag')
    r.add_argument('out')
    r.add_argument('--map')
    r.add_argument('--ns', default='')
    r.add_argument('--ref', default='/odom')
    r.add_argument('--scan', default='/scan')
    r.add_argument('--setpoint', default='/repeat/setpoint')
    r.add_argument('--mount', type=float, nargs=3)
    r.add_argument('--max-range', type=float, default=6.0)
    r.add_argument('--seconds', type=float, default=18)
    r.add_argument('--fps', type=int, default=12)
    r.add_argument('--dpi', type=int, default=70)
    r.add_argument('--title', default='')
    r.add_argument('--still')
    r.add_argument('--es', action='store_true')
    m = sub.add_parser('map')
    m.add_argument('map_dir')
    m.add_argument('out')
    a = ap.parse_args()
    run_animation(a) if a.cmd == 'run' else map_figure(a)


if __name__ == '__main__':
    main()
