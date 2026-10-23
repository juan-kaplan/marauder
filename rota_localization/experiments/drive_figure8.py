#!/usr/bin/env python3
"""
Figure eight in turtlebot3_world: the easy case, with pillars and walls in
view all the time, so ICP never runs out of information.

From the spawn at (-2, -0.5) the robot drives into a figure eight around the
pillars at (-1.1, 0) and (0, 0), radius 0.55 m, and does four circles
(alternating directions), about 15.6 m and 27 rad in all. Open-loop velocity
commands; Gazebo follows them closely enough to stay clear of the pillars.

    python3 drive_figure8.py [--log run.npz]
"""
import math

from driver import run

R, V = 0.55, 0.15
W, T = V / R, 2 * math.pi * R / V


def drive(d):
    d.section('entrada al ocho')
    d.drive_open_loop(V, 0.0, 0.9 / V)
    d.drive_open_loop(V, W, T / 4)
    d.hold(1.0)
    d.report('entrada')
    for k in range(4):
        sense = 'horaria' if k % 2 == 0 else 'antihoraria'
        d.section(f'vuelta {k + 1} ({sense})')
        d.drive_open_loop(V, -W if k % 2 == 0 else W, T)
        d.hold(1.0)
        d.report(f'vuelta {k + 1}')


if __name__ == '__main__':
    run('figure8', drive)
