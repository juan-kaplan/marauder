#!/usr/bin/env python3
"""
Renders a run saved by ``drive_<scenario>.py --log`` as a top-down GIF and a
PNG of the full trajectories, plus the error table in Markdown, named after the
scenario (figure8.gif, corridor.png, ...).

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
from matplotlib.patches import Circle  # noqa: E402
from PIL import Image, ImageSequence  # noqa: E402

from rota_common import se2  # noqa: E402

# Inner faces of the corridor walls (worlds/corridor.world).
CORRIDOR_WALLS = [((-15, -0.8), (12, -0.8)), ((-15, 0.8), (10.4, 0.8)),
                  ((10.4, 0.8), (10.4, 15)), ((12, -0.8), (12, 15))]
# turtlebot3_world: 3 x 3 pillars, radius 0.15 m, 1.1 m apart.
PILLARS = [(x, y) for x in (-1.1, 0.0, 1.1) for y in (-1.1, 0.0, 1.1)]
TITLES = {
    'figure8': 'turtlebot3_world, ocho: trayectoria real y estimada por cada modo',
    'corridor': 'Pasillo en L: trayectoria real y estimada por cada modo',
}
LIDAR_X = -0.032                      # base_footprint -> base_scan on the burger
ANGLES = np.linspace(0.0, 6.28319, 360)
# Mixed goes dashed on top: where ICP has information it matches lidar exactly,
# and a solid line would hide one of the two.
STYLE = {  # label, color, width, alpha, z-order, line style
    'gt': ('real (Gazebo)', '#1a9e3a', 7.0, 0.35, 2, '-'),
    'mixto': ('mixto', '#2762d9', 2.0, 1.0, 7, (0, (4, 2))),
    'lidar': ('lidar', '#e69500', 2.6, 1.0, 6, '-'),
    'encoders': ('encoders', '#d62728', 1.8, 1.0, 5, '-'),
}
STEP = 4          # samples per frame (10 Hz log -> 2.5 frames per sim second)
FPS = 15          # -> 6x real time


def load(path):
    d = dict(np.load(path, allow_pickle=False))
    d['scenario'] = str(d.get('scenario', 'corridor'))   # older logs are corridor runs
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


def setup_axes(ax, scenario, gt, est):
    """Draw the world and fit the view to every trajectory; returns the view's size in m."""
    allx = np.concatenate([gt[:, 0]] + [e[:, 0] for e in est.values()])
    ally = np.concatenate([gt[:, 1]] + [e[:, 1] for e in est.values()])
    if scenario == 'corridor':
        for (x0, y0), (x1, y1) in CORRIDOR_WALLS:
            ax.plot([x0, x1], [y0, y1], color='#555555', lw=3, solid_capstyle='butt')
        lo = (min(allx.min(), -1) - 0.8, min(ally.min(), -0.8) - 0.8)
        hi = (max(allx.max(), 12) + 0.8, max(ally.max(), 4) + 0.8)
    else:
        for c in PILLARS:
            ax.add_patch(Circle(c, 0.15, facecolor='#8a8a8a', edgecolor='#555555', zorder=1))
        lo = (allx.min() - 0.5, ally.min() - 0.5)
        hi = (allx.max() + 0.5, ally.max() + 0.5)
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect('equal')
    ax.grid(color='#e5e5e5', lw=0.6)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    return max(hi[0] - lo[0], hi[1] - lo[1])


def arrow(ax, pose, color, size, z=5):
    return ax.annotate('', xy=(pose[0] + size * math.cos(pose[2]), pose[1] + size * math.sin(pose[2])),
                       xytext=(pose[0], pose[1]), zorder=z,
                       arrowprops=dict(arrowstyle='-|>', color=color, lw=2.2, mutation_scale=16))


def render_gif(d, est, out):
    gt = d['gt']
    fig, ax = plt.subplots(figsize=(10, 6.2), dpi=80)
    fig.subplots_adjust(left=0.07, right=0.98, top=0.9, bottom=0.09)
    span = setup_axes(ax, d['scenario'], gt, est)
    scan_dots, = ax.plot([], [], '.', color='#9a9a9a', ms=2.5, zorder=1)
    lines = {k: ax.plot([], [], color=c, lw=lw, alpha=al, ls=ls, label=lab, zorder=z, solid_capstyle='round')[0]
             for k, (lab, c, lw, al, z, ls) in STYLE.items()}
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
        arrows.append(arrow(ax, gt[i], STYLE['gt'][1], 0.043 * span, z=2))
        rows = []
        for m, e in est.items():
            lines[m].set_data(e[:i + 1, 0], e[:i + 1, 1])
            arrows.append(arrow(ax, e[i], STYLE[m][1], 0.032 * span, z=STYLE[m][4]))
            err = se2.between(tuple(gt[i]), tuple(e[i]))
            rows.append(f'{m:9s}{se2.norm(err) * 100:7.1f} cm {abs(err[2]) * 1000:6.0f} mrad')
        lidar_tag.set_position((est['lidar'][i, 0], est['lidar'][i, 1] - 0.018 * span))
        errors.set_text('error vs real\n' + '\n'.join(rows))
        on = bool(d['lidar_on'][i])
        scan_dots.set_data(*scan_points(gt[i], d['ranges'][i]) if on else ([], []))
        banner.set_visible(not on)
        title.set_text(f"t = {d['t'][i]:5.1f} s   {d['phase'][i]}")
        return []

    FuncAnimation(fig, update, frames=frames).save(out, writer=PillowWriter(fps=FPS))
    plt.close(fig)
    compact_gif(out)


def compact_gif(path, colors=64):
    """
    Re-save with one palette for every frame, so Pillow stores only what changed
    between frames (about a third of the size). The palette comes from frames
    spread over the whole run, so colours seen only briefly (the banner) survive.
    """
    im = Image.open(path)
    duration = im.info.get('duration', round(1000 / FPS))
    frames = [f.convert('RGB') for f in ImageSequence.Iterator(im)]
    samples = frames[::max(1, len(frames) // 12)]
    mosaic = Image.new('RGB', (samples[0].width, samples[0].height * (len(samples) + 1)))
    for k, f in enumerate(samples):
        mosaic.paste(f, (0, k * f.height))
    # A big swatch of each legend colour, so thin arrows keep their exact colour.
    swatch = samples[0].width // len(STYLE)
    for k, (_, color, *_rest) in enumerate(STYLE.values()):
        mosaic.paste(color, (k * swatch, len(samples) * samples[0].height, (k + 1) * swatch, mosaic.height))
    palette = mosaic.quantize(colors=colors, method=Image.Quantize.MEDIANCUT)
    quantized = [f.quantize(palette=palette, dither=Image.Dither.NONE) for f in frames]
    quantized[0].save(path, save_all=True, append_images=quantized[1:], duration=duration, loop=0,
                      optimize=True, disposal=1)


def annotate_corridor(ax, d, gt, est):
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


def render_png(d, est, out):
    gt = d['gt']
    fig, ax = plt.subplots(figsize=(10, 6.2), dpi=110)
    span = setup_axes(ax, d['scenario'], gt, est)
    for k, (lab, c, lw, al, z, ls) in STYLE.items():
        p = gt if k == 'gt' else est[k]
        ax.plot(p[:, 0], p[:, 1], color=c, lw=lw, alpha=al, ls=ls, label=lab, zorder=z, solid_capstyle='round')
        arrow(ax, p[-1], c, (0.043 if k == 'gt' else 0.032) * span, z=z)
    if d['scenario'] == 'corridor':
        annotate_corridor(ax, d, gt, est)
    ax.legend(loc='upper left', fontsize=10, framealpha=0.95)
    ax.set_title(TITLES[d['scenario']])
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
    name = d['scenario']
    render_png(d, est, os.path.join(out_dir, f'{name}.png'))
    with open(os.path.join(out_dir, f'{name}_table.md'), 'w') as f:
        f.write(table_markdown(d))
    render_gif(d, est, os.path.join(out_dir, f'{name}.gif'))
    for suffix in ('.png', '_table.md', '.gif'):
        p = os.path.join(out_dir, name + suffix)
        print(f'{p}  {os.path.getsize(p) / 1e6:.2f} MB')


if __name__ == '__main__':
    main()
