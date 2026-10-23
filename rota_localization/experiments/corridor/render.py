#!/usr/bin/env python3
"""
Renders a run saved by ``drive_corridor.py --log`` as a top-down GIF and a PNG
of the full trajectories, plus the error table in Markdown.

    python3 render.py run.npz OUT_DIR
"""

import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rota_common import se2  # noqa: E402

# Inner faces of the corridor walls (corridor.world).
WALLS = [((-15, -0.8), (12, -0.8)), ((-15, 0.8), (10.4, 0.8)),
         ((10.4, 0.8), (10.4, 15)), ((12, -0.8), (12, 15))]
LIDAR_X = -0.032                      # base_footprint -> base_scan on the burger
ANGLES = np.linspace(0.0, 6.28319, 360)
STYLE = {  # label, color, width, alpha, z-order
    'gt': ('real (Gazebo)', '#1a9e3a', 7.0, 0.35, 2),
    'mixto': ('mixto', '#2762d9', 1.8, 1.0, 4),
    'lidar': ('lidar', '#e69500', 2.6, 1.0, 6),
    'encoders': ('encoders', '#d62728', 1.8, 1.0, 5),
}
STEP = 3          # samples per frame (10 Hz log -> ~3.3 frames per sim second)
FPS = 15          # -> ~4.5x real time


def load(path):
    d = np.load(path, allow_pickle=False)
    gt0 = tuple(d['gt0'])
    est = {}
    for k, m in enumerate(d['modes']):
        # Same as RViz: the estimate's origin sits where the robot started.
        e0 = tuple(d['est0'][k])
        est[str(m)] = np.array([se2.compose(gt0, se2.between(e0, tuple(p))) for p in d['est'][:, k]])
    return d, est


def scan_points(pose, ranges):
    ok = np.isfinite(ranges)
    r, a = ranges[ok], ANGLES[ok]
    lidar = se2.compose(pose, (LIDAR_X, 0.0, 0.0))
    c, s = math.cos(lidar[2]), math.sin(lidar[2])
    px, py = r * np.cos(a), r * np.sin(a)
    return lidar[0] + c * px - s * py, lidar[1] + s * px + c * py


def setup_axes(ax, gt, est):
    for (x0, y0), (x1, y1) in WALLS:
        ax.plot([x0, x1], [y0, y1], color='#555555', lw=3, solid_capstyle='butt')
    allx = np.concatenate([gt[:, 0]] + [e[:, 0] for e in est.values()])
    ally = np.concatenate([gt[:, 1]] + [e[:, 1] for e in est.values()])
    ax.set_xlim(min(allx.min(), -1) - 0.8, max(allx.max(), 12) + 0.8)
    ax.set_ylim(min(ally.min(), -0.8) - 0.8, max(ally.max(), 4) + 0.8)
    ax.set_aspect('equal')
    ax.grid(color='#e5e5e5', lw=0.6)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')


def arrow(ax, pose, color, size=0.45, z=5):
    return ax.annotate('', xy=(pose[0] + size * math.cos(pose[2]), pose[1] + size * math.sin(pose[2])),
                       xytext=(pose[0], pose[1]), zorder=z,
                       arrowprops=dict(arrowstyle='-|>', color=color, lw=2.2, mutation_scale=16))


def render_gif(d, est, out):
    gt = d['gt']
    fig, ax = plt.subplots(figsize=(10, 6.2), dpi=80)
    fig.subplots_adjust(left=0.07, right=0.98, top=0.9, bottom=0.09)
    setup_axes(ax, gt, est)
    scan_dots, = ax.plot([], [], '.', color='#9a9a9a', ms=2.5, zorder=1)
    lines = {k: ax.plot([], [], color=c, lw=lw, alpha=al, label=lab, zorder=z, solid_capstyle='round')[0]
             for k, (lab, c, lw, al, z) in STYLE.items()}
    ax.legend(loc='upper left', fontsize=10, framealpha=0.95)
    title = fig.suptitle('', fontsize=13, x=0.07, ha='left')
    errors = ax.text(0.01, 0.70, '', transform=ax.transAxes, ha='left', va='top', family='monospace',
                     fontsize=10, bbox=dict(facecolor='white', edgecolor='#cccccc', alpha=0.95), zorder=10)
    lidar_tag = ax.text(0, 0, 'lidar', color=STYLE['lidar'][1], fontsize=10, weight='bold', zorder=7,
                        ha='center', va='top')
    banner = ax.text(0.5, 0.92, 'LiDAR TAPADO', transform=ax.transAxes, ha='center', fontsize=16,
                     color='white', weight='bold', bbox=dict(facecolor='#c0392b', edgecolor='none'))
    arrows = []
    frames = range(0, len(gt), STEP)

    def update(i):
        for a in arrows:
            a.remove()
        arrows.clear()
        lines['gt'].set_data(gt[:i + 1, 0], gt[:i + 1, 1])
        arrows.append(arrow(ax, gt[i], STYLE['gt'][1], size=0.6, z=2))
        rows = []
        for m, e in est.items():
            lines[m].set_data(e[:i + 1, 0], e[:i + 1, 1])
            arrows.append(arrow(ax, e[i], STYLE[m][1], z=STYLE[m][4]))
            err = se2.between(tuple(gt[i]), tuple(e[i]))
            rows.append(f'{m:9s}{se2.norm(err) * 100:7.1f} cm {abs(err[2]) * 1000:6.0f} mrad')
        lidar_tag.set_position((est['lidar'][i, 0], est['lidar'][i, 1] - 0.25))
        errors.set_text('error vs real\n' + '\n'.join(rows))
        on = bool(d['lidar_on'][i])
        scan_dots.set_data(*scan_points(gt[i], d['ranges'][i]) if on else ([], []))
        banner.set_visible(not on)
        title.set_text(f"t = {d['t'][i]:5.1f} s   {d['phase'][i]}")
        return []

    FuncAnimation(fig, update, frames=frames).save(out, writer=PillowWriter(fps=FPS))
    plt.close(fig)


def render_png(d, est, out):
    gt = d['gt']
    fig, ax = plt.subplots(figsize=(10, 6.2), dpi=110)
    setup_axes(ax, gt, est)
    for k, (lab, c, lw, al, z) in STYLE.items():
        p = gt if k == 'gt' else est[k]
        ax.plot(p[:, 0], p[:, 1], color=c, lw=lw, alpha=al, label=lab, zorder=z, solid_capstyle='round')
        arrow(ax, p[-1], c, size=0.6 if k == 'gt' else 0.45, z=z)
    blind = np.flatnonzero(~d['lidar_on'])
    if blind.size:
        ax.annotate('LiDAR tapado,\ngira 90°', xy=gt[blind[0], :2], xytext=(gt[blind[0], 0] - 3.2, 2.2),
                    arrowprops=dict(arrowstyle='->', color='#c0392b'), color='#c0392b', fontsize=10)
    ax.annotate('stop and go: nada al alcance del LiDAR\nadelante ni atrás (0 a 4.4 m)', xy=(4.4, 0.0),
                xytext=(4.6, -1.35), va='center', arrowprops=dict(arrowstyle='->', color='#555555'),
                color='#333333', fontsize=10)
    end = est['lidar'][-1]
    ax.annotate('donde termina creyendo\nestar el lidar', xy=end[:2], xytext=(end[0] - 0.6, 2.2),
                arrowprops=dict(arrowstyle='->', color=STYLE['lidar'][1]), color='#a86d00', fontsize=10)
    ax.legend(loc='upper left', fontsize=10, framealpha=0.95)
    ax.set_title('Pasillo en L: trayectoria real y estimada por cada modo')
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def table_markdown(d):
    modes = [str(m) for m in d['modes']]
    rows = {}
    for label, m, pos, ang in d['table']:
        rows.setdefault(str(label), {})[str(m)] = (float(pos), float(ang))
    lines = ['| Momento | ' + ' | '.join(modes) + ' |', '|---|' + '---|' * len(modes)]
    for label, by_mode in rows.items():
        cells = [f'{by_mode[m][0] * 100:.1f} cm / {by_mode[m][1] * 1000:.0f} mrad' for m in modes]
        lines.append(f'| {label} | ' + ' | '.join(cells) + ' |')
    return '\n'.join(lines) + '\n'


def main():
    run, out_dir = sys.argv[1], sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)
    d, est = load(run)
    render_png(d, est, os.path.join(out_dir, 'corridor.png'))
    with open(os.path.join(out_dir, 'corridor_table.md'), 'w') as f:
        f.write(table_markdown(d))
    render_gif(d, est, os.path.join(out_dir, 'corridor.gif'))
    for name in ('corridor.png', 'corridor_table.md', 'corridor.gif'):
        p = os.path.join(out_dir, name)
        print(f'{p}  {os.path.getsize(p) / 1e6:.2f} MB')


if __name__ == '__main__':
    main()
