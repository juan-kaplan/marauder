#!/usr/bin/env python3
"""
L corridor: where ICP alone runs out of information.

1. stop and go along leg 1 (nothing within LiDAR range ahead or behind),
2. on to the corner, then the LiDAR goes dark while the robot turns 90 deg,
3. along leg 2.

    python3 drive_corridor.py [--log run.npz]
"""
import math

from driver import run

CORNER_X = 11.2


def drive(d):
    leg1, leg2 = ((0.0, 0.0), 0.0), ((CORNER_X, 0.0), math.pi / 2)

    d.section('1) pasillo: stop and go')
    for v, t in [(0.2, 3), (0, 2), (0.25, 3), (0.05, 4), (0.22, 3), (0, 2)] * 2:
        d.drive_for(v, t, *leg1)
    d.report('tras stop and go')
    d.section('1) pasillo: hasta la esquina')
    d.drive_until(0.2, *leg1, done=lambda p: p[0] >= CORNER_X - 0.6)
    d.drive_until(0.08, *leg1, done=lambda p: p[0] >= CORNER_X)
    d.hold(1.0)
    d.report('en la esquina')

    d.section('2) LiDAR tapado, gira 90°')
    d.lidar_on = False
    d.hold(0.5)
    d.turn_to(math.pi / 2)
    d.hold(0.5)
    d.lidar_on = True
    d.phase = '2) vuelve el LiDAR'
    d.hold(1.5)
    d.report('tras el giro a ciegas')

    d.section('3) segundo tramo')
    d.drive_until(0.2, *leg2, done=lambda p: p[1] >= 4.0)
    d.hold(1.5)
    d.report('final')


if __name__ == '__main__':
    run('corridor', drive)
