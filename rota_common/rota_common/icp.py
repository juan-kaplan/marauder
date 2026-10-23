"""
Planar ICP between two ``(N, 2)`` point sets (Open3D underneath).

Two error metrics:

* ``point_to_plane`` (default; in 2D, point-to-line): distance from each
  source point to the line through its target neighbour. Sliding along a wall
  costs nothing, so the different sampling of the two scans does not bias the
  result.
* ``point_to_point``: distance between matched points. It stops within about
  half the point spacing of the truth, because points settle on a neighbouring
  sample of the same wall. Kept for comparison.
"""

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np
import open3d as o3d

from rota_common.se2 import Pose2D, from_matrix, to_matrix

# Below this many points per cloud the alignment is not even attempted.
MIN_POINTS = 10
METHODS = ('point_to_plane', 'point_to_point')


@dataclass
class IcpParams:
    max_correspondence_distance: float = 0.30
    max_iterations: int = 30
    min_fitness: float = 0.25
    method: str = 'point_to_plane'
    normal_neighbors: int = 8  # target neighbours used to fit each line (point_to_plane)


@dataclass
class IcpResult:
    pose: Pose2D    # pose of the source robot in the target robot's frame
    fitness: float  # fraction of source points with a correspondence (0..1)
    rmse: float     # RMS distance of those correspondences (inf if none)
    ok: bool        # fitness >= min_fitness


def _cloud(points: np.ndarray) -> o3d.geometry.PointCloud:
    pts = np.zeros((len(points), 3))
    pts[:, :2] = points
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd


def _set_line_normals(pcd: o3d.geometry.PointCloud, neighbors: int):
    """
    Give each point the in-plane normal of the line through its neighbours.

    Open3D's own normal estimation returns +z for points on a plane, so the
    normal is the eigenvector of the smallest eigenvalue of the 2x2 covariance.
    """
    pcd.estimate_covariances(o3d.geometry.KDTreeSearchParamKNN(neighbors))
    cov = np.asarray(pcd.covariances)[:, :2, :2]
    _, vecs = np.linalg.eigh(cov)
    normals = np.zeros((len(cov), 3))
    normals[:, :2] = vecs[:, :, 0]
    pcd.normals = o3d.utility.Vector3dVector(normals)


def align(source: np.ndarray, target: np.ndarray, guess: Sequence[float],
          params: IcpParams = IcpParams()) -> IcpResult:
    """
    Align ``source`` to ``target``, both in their own robot frame.

    ``guess`` is the initial estimate of the source robot's pose in the target
    robot's frame. When ICP cannot run or does not reach ``min_fitness`` the
    result has ``ok=False``; if it could not run at all, ``pose`` is ``guess``.
    """
    if params.method not in METHODS:
        raise ValueError(f"ICP method must be one of {METHODS}, got '{params.method}'")
    guess = tuple(float(v) for v in guess)
    source = np.asarray(source, dtype=np.float64).reshape(-1, 2)
    target = np.asarray(target, dtype=np.float64).reshape(-1, 2)
    if len(source) < MIN_POINTS or len(target) < MIN_POINTS:
        return IcpResult(guess, 0.0, math.inf, False)

    target_pcd = _cloud(target)
    if params.method == 'point_to_plane':
        _set_line_normals(target_pcd, params.normal_neighbors)
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    else:
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()

    reg = o3d.pipelines.registration.registration_icp(
        source=_cloud(source),
        target=target_pcd,
        max_correspondence_distance=params.max_correspondence_distance,
        init=to_matrix(guess),
        estimation_method=estimation,
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=params.max_iterations))

    if not np.all(np.isfinite(reg.transformation)):
        return IcpResult(guess, 0.0, math.inf, False)
    fitness = float(reg.fitness)
    rmse = float(reg.inlier_rmse) if fitness > 0.0 else math.inf
    return IcpResult(from_matrix(reg.transformation), fitness, rmse, fitness >= params.min_fitness)
