"""
Drives the TB3 through the L corridor like a careful human (steering on Gazebo's
ground truth, which the localizers never see) and gates the LiDAR:

1. stop and go along leg 1 (nothing within LiDAR range ahead or behind),
2. on to the corner, then the LiDAR goes dark while the robot turns 90 deg,
3. along leg 2.

Prints each localizer's error against the ground truth at every stop. With
--log FILE.npz it also saves, at 10 Hz of sim time, the true pose, the three
estimates, whether the LiDAR is blocked and the scan ranges, for render.py.

    python3 drive_corridor.py [--log run.npz]
"""
import argparse
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from rota_common import se2
from rota_common.msgs import pose_from_msg

CORNER_X = 11.2
LOG_PERIOD = 0.1   # s of sim time
MODES = {'mixto': '/localization/odom', 'lidar': '/lidar/localization/odom',
         'encoders': '/encoders/localization/odom'}


class Driver:
    def __init__(self):
        self.n = rclpy.create_node('corridor_driver', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.cmd = self.n.create_publisher(Twist, '/cmd_vel', 10)
        self.gated = self.n.create_publisher(LaserScan, '/scan_gated', qos_profile_sensor_data)
        self.lidar_on = True
        self.gt = None
        self.scan = None
        self.log = []
        self.rows = []
        self.phase = ''
        self.next_log = 0.0
        self.est = {m: None for m in MODES}
        self.n.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.n.create_subscription(Odometry, '/ground_truth/odom', lambda m: setattr(self, 'gt', pose_from_msg(m)), 10)
        for m, topic in MODES.items():
            self.n.create_subscription(Odometry, topic,
                                       lambda msg, m=m: self.est.__setitem__(m, pose_from_msg(msg)), 10)

    def on_scan(self, msg):
        self.scan = msg
        if self.lidar_on:
            self.gated.publish(msg)

    def now(self):
        return self.n.get_clock().now().nanoseconds * 1e-9

    def spin(self, v=0.0, w=0.0):
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        self.cmd.publish(m)
        rclpy.spin_once(self.n, timeout_sec=0.03)
        t = self.now()
        if self.gt is not None and t >= self.next_log and hasattr(self, 'gt0'):
            self.next_log = t + LOG_PERIOD
            ranges = np.full(360, np.nan)
            if self.lidar_on and self.scan is not None and len(self.scan.ranges) == 360:
                ranges = np.asarray(self.scan.ranges, dtype=float)
            self.log.append((t, self.gt, [self.est[m] for m in MODES], self.lidar_on, ranges, self.phase))

    def steer(self, lane, heading):
        """Angular rate that keeps the robot on the line through ``lane`` with ``heading``."""
        x, y, th = self.gt
        lateral = math.cos(heading) * (y - lane[1]) - math.sin(heading) * (x - lane[0])
        return max(-0.8, min(0.8, -2.0 * lateral - 2.0 * se2.normalize_angle(th - heading)))

    def hold(self, seconds):
        end = self.now() + seconds
        while self.now() < end:
            self.spin()

    def drive_for(self, v, seconds, lane, heading):
        end = self.now() + seconds
        while self.now() < end:
            self.spin(v, self.steer(lane, heading) if v > 0 else 0.0)

    def drive_until(self, v, lane, heading, done):
        while not done(self.gt):
            self.spin(v, self.steer(lane, heading))

    def turn_to(self, heading, w_max=0.6):
        while abs(se2.normalize_angle(heading - self.gt[2])) > 0.02:
            e = se2.normalize_angle(heading - self.gt[2])
            self.spin(0.0, max(-w_max, min(w_max, 2.5 * e)))

    def report(self, label):
        g = se2.between(self.gt0, self.gt)
        row = []
        for m in MODES:
            e = se2.between(g, se2.between(self.est0[m], self.est[m]))
            row.append(f'{m} {se2.norm(e) * 100:6.1f} cm {abs(e[2]) * 1000:5.0f} mrad')
            self.rows.append((label.strip(), m, se2.norm(e), abs(e[2])))
        print(f'{label:28s} | ' + ' | '.join(row), flush=True)

    def save(self, path):
        t, gt, est, lidar_on, ranges, phase = zip(*self.log)
        np.savez_compressed(
            path, t=np.array(t) - t[0], gt=np.array(gt), est=np.array(est), modes=np.array(list(MODES)),
            gt0=np.array(self.gt0), est0=np.array([self.est0[m] for m in MODES]),
            lidar_on=np.array(lidar_on), ranges=np.array(ranges), phase=np.array(phase),
            table=np.array([(r[0], r[1], f'{r[2]:.6f}', f'{r[3]:.6f}') for r in self.rows]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--log', help='save the run to this .npz for render.py')
    args = parser.parse_args()
    rclpy.init()
    d = Driver()
    while d.now() == 0.0 or d.gt is None or any(v is None for v in d.est.values()):
        rclpy.spin_once(d.n, timeout_sec=0.1)
    d.hold(2.0)
    d.gt0, d.est0 = d.gt, dict(d.est)
    leg1, leg2 = ((0.0, 0.0), 0.0), ((CORNER_X, 0.0), math.pi / 2)

    print('1) pasillo, stop and go', flush=True)
    d.phase = '1) pasillo: stop and go'
    for v, t in [(0.2, 3), (0, 2), (0.25, 3), (0.05, 4), (0.22, 3), (0, 2)] * 2:
        d.drive_for(v, t, *leg1)
    d.report('   tras stop and go')
    d.phase = '1) pasillo: hasta la esquina'
    d.drive_until(0.2, *leg1, done=lambda p: p[0] >= CORNER_X - 0.6)
    d.drive_until(0.08, *leg1, done=lambda p: p[0] >= CORNER_X)
    d.hold(1.0)
    d.report('   en la esquina')

    print('2) LiDAR tapado mientras dobla 90 grados', flush=True)
    d.phase = '2) LiDAR tapado, gira 90°'
    d.lidar_on = False
    d.hold(0.5)
    d.turn_to(math.pi / 2)
    d.hold(0.5)
    d.lidar_on = True
    d.phase = '2) vuelve el LiDAR'
    d.hold(1.5)
    d.report('   tras el giro a ciegas')

    print('3) segundo tramo', flush=True)
    d.phase = '3) segundo tramo'
    d.drive_until(0.2, *leg2, done=lambda p: p[1] >= 4.0)
    d.hold(1.5)
    d.report('   final')
    d.spin()
    if args.log:
        d.save(args.log)
        print(f'saved {len(d.log)} samples to {args.log}', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
