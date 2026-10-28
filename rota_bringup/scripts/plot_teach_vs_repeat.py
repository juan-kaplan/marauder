#!/usr/bin/env python3
"""
Plot a Teach against its Repeat from two bags and, optionally, the Teach's map.

  python3 plot_teach_vs_repeat.py teach_bag repeat_bag out.png [--map map_dir] [--ref /odom] [--loc /localization/odom] [--es]

Both bags need the reference topic (Gazebo's /odom, or /tb4_1/odom on the
robot) and the localizer's odom. Without --map no keyframes are drawn. --es
labels the figure in Spanish. Run it with ROS sourced, so it can read bags.
"""

import argparse
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
import rosbag2_py


def read_odom(bag, topic):
    """Return (t, x, y, yaw) from the header stamps of an Odometry topic."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('', ''))
    rows = []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name != topic:
            continue
        m = deserialize_message(data, Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, yaw))
    if not rows:
        raise SystemExit(f'no {topic} in {bag}')
    return np.array(rows).T


def to_frame(origin, x, y):
    """Points seen from the pose origin = (x, y, yaw)."""
    c, s = np.cos(origin[2]), np.sin(origin[2])
    dx, dy = x - origin[0], y - origin[1]
    return c * dx + s * dy, -s * dx + c * dy


def from_frame(origin, x, y):
    c, s = np.cos(origin[2]), np.sin(origin[2])
    return origin[0] + c * x - s * y, origin[1] + s * x + c * y


def dist_to_path(px, py, x, y):
    """Distance from each point (px, py) to the polyline (x, y)."""
    ax, ay, bx, by = x[:-1], y[:-1], x[1:], y[1:]
    abx, aby = bx - ax, by - ay
    ab2 = np.maximum(abx ** 2 + aby ** 2, 1e-12)
    out = []
    for qx, qy in zip(px, py):
        u = np.clip(((qx - ax) * abx + (qy - ay) * aby) / ab2, 0, 1)
        out.append(np.min(np.hypot(qx - ax - u * abx, qy - ay - u * aby)))
    return np.array(out)


def length(x, y):
    return float(np.sum(np.hypot(np.diff(x), np.diff(y))))


LABELS = {
    'en': ('taught (reference)', 'repeated (reference)', 'repeated (localizer)', 'keyframes',
           'time since the Repeat started moving (s)', 'distance to the taught path (m)',
           'mean {:.3f} m, max {:.3f} m', 'end error'),
    'es': ('enseñado (referencia)', 'repetido (referencia)', 'repetido (localizador)', 'keyframes',
           'tiempo desde que la repetición empezó a moverse (s)', 'distancia a la trayectoria enseñada (m)',
           'media {:.3f} m, máx. {:.3f} m', 'error final'),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('teach_bag')
    ap.add_argument('repeat_bag')
    ap.add_argument('out')
    ap.add_argument('--map')
    ap.add_argument('--es', action='store_true')
    ap.add_argument('--ref', default='/odom')
    ap.add_argument('--loc', default='/localization/odom')
    ap.add_argument('--title', default='')
    a = ap.parse_args()

    tt, tx, ty, tyaw = read_odom(a.teach_bag, a.ref)
    rt, rx, ry, ryaw = read_odom(a.repeat_bag, a.ref)
    lt, lx, ly, lyaw = read_odom(a.repeat_bag, a.loc)
    kfs = None
    if a.map:
        with open(os.path.join(a.map, 'map.json')) as f:
            kfs = np.array([k['pose_odom'] for k in json.load(f)['keyframes']])

    # Everything in the frame of the Teach's first reference pose.
    origin = (tx[0], ty[0], tyaw[0])
    tx, ty = to_frame(origin, tx, ty)
    rx, ry = to_frame(origin, rx, ry)
    ryaw = ryaw - origin[2]
    # The localizer's odom starts at the identity where the Repeat started.
    i = np.searchsorted(rt, lt[0])
    lx, ly = from_frame((rx[i], ry[i], ryaw[i]), lx, ly)

    # Time zero: the first time the repeated robot has moved 2 cm.
    moved = np.nonzero(np.hypot(rx - rx[0], ry - ry[0]) > 0.02)[0]
    t0 = rt[moved[0]] if len(moved) else rt[0]
    keep = rt >= t0
    d = dist_to_path(rx[keep], ry[keep], tx, ty)
    end_err = float(np.hypot(rx[-1] - tx[-1], ry[-1] - ty[-1]))
    mean, peak = float(d.mean()), float(d.max())
    print(f'taught length {length(tx, ty):.2f} m, repeated length {length(rx[keep], ry[keep]):.2f} m, '
          f'{0 if kfs is None else len(kfs)} keyframes')
    print(f'distance to the taught path: mean {mean:.3f} m, max {peak:.3f} m; end error {end_err:.3f} m')

    L = LABELS['es' if a.es else 'en']
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.8), gridspec_kw={'width_ratios': [1.2, 1]})
    ax1.plot(tx, ty, 'g-', lw=2.5, label=L[0])
    ax1.plot(rx[keep], ry[keep], 'b-', lw=1.5, label=L[1])
    ax1.plot(lx, ly, 'r--', lw=1, label=L[2])
    if kfs is not None:
        ax1.plot(kfs[:, 0], kfs[:, 1], 'k.', ms=7, label=L[3])
    ax1.plot(tx[-1], ty[-1], 'gs', ms=8)
    ax1.plot(rx[-1], ry[-1], 'bx', ms=9)
    ax1.set_aspect('equal')
    ax1.grid(alpha=0.3)
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.legend(fontsize=8)
    ax2.plot(rt[keep] - t0, d, 'b-')
    ax2.grid(alpha=0.3)
    ax2.set_xlabel(L[4])
    ax2.set_ylabel(L[5])
    ax2.set_title(L[6].format(mean, peak), fontsize=10)
    fig.suptitle(f'{a.title} {L[7]} {end_err:.3f} m'.strip())
    fig.tight_layout()
    fig.savefig(a.out, dpi=130)


if __name__ == '__main__':
    main()
