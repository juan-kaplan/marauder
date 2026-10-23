"""
Planar point-to-plane ICP between two (N, 2) point sets (Open3D underneath).
Point-to-plane is point-to-line in 2D: sliding along a wall costs nothing, so
the different sampling of the two scans does not bias the result.
"""

import os

import numpy as np

# Open3D's OpenMP threads make a 0.5 ms alignment take hundreds of ms when
# several nodes run ICP at once. Only works if nothing imported open3d before.
os.environ.setdefault('OMP_NUM_THREADS', '1')
import open3d as o3d  # noqa: E402

from rota_common.se2 import from_matrix, to_matrix  # noqa: E402


def cloud(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.column_stack((points, np.zeros(len(points)))))
    return pcd


def align(source, target, guess, max_distance=0.30, max_iterations=30, min_fitness=0.25):
    """
    Pose of the source robot in the target robot's frame, starting from guess.
    Returns (pose, fitness, ok); the pose is the guess if ICP could not run.
    """
    if len(source) < 10 or len(target) < 10:
        return tuple(guess), 0.0, False

    target_pcd = cloud(target)
    # Open3D's normals point along z for a planar cloud, so take the in-plane
    # normal from the covariance of each point's neighbours.
    target_pcd.estimate_covariances(o3d.geometry.KDTreeSearchParamKNN(8))
    _, vecs = np.linalg.eigh(np.asarray(target_pcd.covariances)[:, :2, :2])
    normals = np.zeros((len(target), 3))
    normals[:, :2] = vecs[:, :, 0]
    target_pcd.normals = o3d.utility.Vector3dVector(normals)

    reg = o3d.pipelines.registration.registration_icp(
        cloud(source), target_pcd, max_distance, to_matrix(guess),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iterations))
    if not np.all(np.isfinite(reg.transformation)):
        return tuple(guess), 0.0, False
    return from_matrix(reg.transformation), reg.fitness, reg.fitness >= min_fitness
