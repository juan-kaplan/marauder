"""
Cases where ICP alone struggles and the mixed mode has to earn its keep.

Realistic settings: scans at 5 Hz (TB3 LDS), 1 cm range noise, and wheels with
a 0.5% radius difference plus random error, so the mixed mode is not handed
perfect encoders.
"""

import numpy as np

from rota_common import se2
from rota_localization.localizer import EncoderNoise, Localizer
from sim import CORRIDOR, ROOM, drive

REALISTIC = dict(scan_hz=5, scan_noise=0.01)


def run(mode, commands, **kw):
    loc = Localizer(mode, encoder_noise=EncoderNoise(std=0.002, bias=0.005, seed=1))
    trace = []
    drive(loc, commands, trace=trace, scans=mode != 'encoders', **REALISTIC, **kw)
    errors = [se2.between(truth, est) for truth, est in trace]
    final = errors[-1]
    worst = max(se2.norm(e) for e in errors)
    return se2.norm(final), abs(final[2]), worst


# Stop and go along a corridor longer than the LiDAR range.
STOP_AND_GO = [(0.2, 0.0, 3.0), (0.0, 0.0, 2.0), (0.25, 0.0, 3.0), (0.05, 0.0, 4.0),
               (0.22, 0.0, 3.0), (0.0, 0.0, 2.0)]   # 2.2 m in total


def test_corridor_lidar_cannot_see_progress():
    # Every scan looks the same along the corridor: lidar mode never sees the
    # robot advance. This documents the limitation, it is not a bug.
    pos, _, _ = run('lidar', STOP_AND_GO, segments=CORRIDOR)
    assert pos > 1.5


def test_corridor_mixed_takes_progress_from_the_wheels():
    pos, ang, worst = run('mixed', STOP_AND_GO, segments=CORRIDOR)
    assert pos < 0.02 and ang < 0.01 and worst < 0.02
    # ...and the heading still comes from ICP, so it beats the wheels alone.
    enc_pos, enc_ang, _ = run('encoders', STOP_AND_GO, segments=CORRIDOR)
    assert ang < enc_ang / 5 and pos < enc_pos


# Straight, then a turn while the LiDAR is blocked for 2 s, then straight.
BLOCKED_TURN = [(0.2, 0.0, 3.0), (0.15, 0.6, 2.0), (0.2, 0.0, 3.0)]


def test_lidar_blocked_while_turning():
    kw = dict(segments=ROOM, start=(0.0, 0.5, 0.0), scan_gap=(2.9, 5.1))
    lidar_pos, lidar_ang, _ = run('lidar', BLOCKED_TURN, **kw)
    mixed_pos, mixed_ang, mixed_worst = run('mixed', BLOCKED_TURN, **kw)
    # After the gap the constant-velocity guess is 1.2 rad off and ICP locks
    # onto the wrong walls; the wheels carry the mixed mode through it.
    assert lidar_ang > 0.5
    assert mixed_pos < 0.05 and mixed_ang < 0.05 and mixed_worst < 0.05
    assert np.hypot(mixed_pos, mixed_ang) < np.hypot(lidar_pos, lidar_ang) / 10
