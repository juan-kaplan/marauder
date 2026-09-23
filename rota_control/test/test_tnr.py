#!/usr/bin/env python3

import json
import math
import os
import tempfile
import numpy as np
import pytest

from rota_control.teach_node import (
    normalize_angle as teach_norm,
    se2_minus as teach_se2_minus,
    laser_scan_to_points
)
from rota_control.repeat_node import (
    normalize_angle as repeat_norm,
    se2_minus as repeat_se2_minus,
    se2_to_matrix4,
    matrix4_to_se2,
    points_to_open3d_pcd,
)
import open3d as o3d


def test_angle_normalization():
    for norm in [teach_norm, repeat_norm]:
        assert math.isclose(norm(0.0), 0.0, abs_tol=1e-6)
        assert math.isclose(norm(math.pi), math.pi, abs_tol=1e-6)
        assert math.isclose(norm(-math.pi), math.pi, abs_tol=1e-6)
        assert math.isclose(norm(3.0 * math.pi), math.pi, abs_tol=1e-6)
        assert math.isclose(norm(-3.0 * math.pi), math.pi, abs_tol=1e-6)
        assert math.isclose(norm(math.pi / 2.0), math.pi / 2.0, abs_tol=1e-6)
        assert math.isclose(norm(-math.pi / 2.0), -math.pi / 2.0, abs_tol=1e-6)


def test_se2_minus():
    # Identical poses should have zero difference
    pose_a = (1.0, 2.0, 0.5)
    rel = teach_se2_minus(pose_a, pose_a)
    assert math.isclose(rel[0], 0.0, abs_tol=1e-6)
    assert math.isclose(rel[1], 0.0, abs_tol=1e-6)
    assert math.isclose(rel[2], 0.0, abs_tol=1e-6)

    # Pure translation along x with heading 0
    pose_ref = (0.0, 0.0, 0.0)
    pose_target = (2.0, 0.0, 0.0)
    rel = repeat_se2_minus(pose_target, pose_ref)
    assert math.isclose(rel[0], 2.0, abs_tol=1e-6)
    assert math.isclose(rel[1], 0.0, abs_tol=1e-6)
    assert math.isclose(rel[2], 0.0, abs_tol=1e-6)

    # Translation with reference rotated 90 degrees (pi/2)
    # Target is at (0, 2) in world. In reference frame (facing +y), it is 2m in front (along local x).
    pose_ref = (0.0, 0.0, math.pi / 2.0)
    pose_target = (0.0, 2.0, math.pi / 2.0)
    rel = repeat_se2_minus(pose_target, pose_ref)
    assert math.isclose(rel[0], 2.0, abs_tol=1e-6)
    assert math.isclose(rel[1], 0.0, abs_tol=1e-6)
    assert math.isclose(rel[2], 0.0, abs_tol=1e-6)


def test_se2_matrix_roundtrip():
    x, y, theta = 1.25, -0.75, 0.85
    T = se2_to_matrix4(x, y, theta)
    xr, yr, thetar = matrix4_to_se2(T)
    assert math.isclose(xr, x, abs_tol=1e-6)
    assert math.isclose(yr, y, abs_tol=1e-6)
    assert math.isclose(thetar, theta, abs_tol=1e-6)


def test_open3d_icp_alignment():
    # Create a synthetic L-shaped room corner
    pts = []
    for d in np.linspace(0.5, 3.0, 50):
        pts.append([d, 0.0])  # wall 1 along x
        pts.append([0.0, d])  # wall 2 along y
    pts = np.array(pts)

    # Reference point cloud (anchor)
    pcd_target = points_to_open3d_pcd(pts)

    # Simulated displacement: robot moved 0.15m in x, 0.05m in y, rotated 0.08 rad (~4.5 deg)
    true_dx, true_dy, true_dtheta = 0.15, 0.05, 0.08
    T_true = se2_to_matrix4(true_dx, true_dy, true_dtheta)

    # Source points in robot frame (robot sees corner shifted by inverse of its motion)
    T_inv = np.linalg.inv(T_true)
    pcd_source = o3d.geometry.PointCloud(pcd_target)
    pcd_source.transform(T_inv)

    # Run ICP with an odometry guess (with slight noise)
    T_guess = se2_to_matrix4(true_dx + 0.02, true_dy - 0.02, true_dtheta + 0.02)
    reg = o3d.pipelines.registration.registration_icp(
        source=pcd_source,
        target=pcd_target,
        max_correspondence_distance=0.3,
        init=T_guess,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
    )

    est_x, est_y, est_theta = matrix4_to_se2(reg.transformation)
    assert reg.fitness > 0.8
    assert math.isclose(est_x, true_dx, abs_tol=0.01)
    assert math.isclose(est_y, true_dy, abs_tol=0.01)
    assert math.isclose(est_theta, true_dtheta, abs_tol=0.01)


def test_trajectory_serialization():
    with tempfile.TemporaryDirectory() as tmpdir:
        json_path = os.path.join(tmpdir, "trajectory.json")
        npz_path = os.path.join(tmpdir, "trajectory_scans.npz")

        keyframes = [
            {'id': 0, 'stamp': 100.0, 'pose': [0.0, 0.0, 0.0], 'rel_transform': [0.0, 0.0, 0.0], 'num_points': 10},
            {'id': 1, 'stamp': 101.5, 'pose': [0.3, 0.0, 0.0], 'rel_transform': [0.3, 0.0, 0.0], 'num_points': 12},
        ]
        meta = {
            'tau_d': 0.3,
            'tau_theta': 0.3,
            'total_keyframes': len(keyframes),
            'keyframes': keyframes
        }
        with open(json_path, 'w') as f:
            json.dump(meta, f)

        scans = {
            'scan_0': np.random.rand(10, 2),
            'scan_1': np.random.rand(12, 2),
        }
        np.savez_compressed(npz_path, **scans)

        # Reload
        with open(json_path, 'r') as f:
            loaded_meta = json.load(f)
        loaded_scans = np.load(npz_path)

        assert loaded_meta['total_keyframes'] == 2
        assert len(loaded_meta['keyframes']) == 2
        assert 'scan_0' in loaded_scans
        assert 'scan_1' in loaded_scans
        assert loaded_scans['scan_0'].shape == (10, 2)
