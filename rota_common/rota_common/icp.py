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

Degenerate geometry (a corridor, a single wall): along some directions the
scans carry almost no information, and what ICP returns there is set by noise
and by how the beams happen to sample the walls. In a corridor every scan looks
the same in the robot frame, so ICP tends to report no motion along it. Those
directions are found from the eigenvalues of the point-to-line information
matrix, and the result keeps the guess there (solution remapping, Zhang, Kaess
and Singh, ICRA 2016).
"""

import math
import os
from dataclasses import dataclass
from typing import Sequence

import numpy as np

# Open3D parallelises with OpenMP. For a few hundred points the threads cost
# more than they save, and with several nodes running ICP at once
# (localization, teach, repeat) they spin against each other: an alignment
# that takes 0.5 ms went to ~200 ms with three processes. It only takes
# effect if nothing imported open3d earlier in the process.
os.environ.setdefault('OMP_NUM_THREADS', '1')
import open3d as o3d  # noqa: E402

from rota_common.se2 import Pose2D, compose, from_matrix, inverse, to_matrix  # noqa: E402

# Below this many points per cloud the alignment is not even attempted.
MIN_POINTS = 10
METHODS = ('point_to_plane', 'point_to_point')


@dataclass
class IcpParams:
    max_correspondence_distance: float = 0.30
    max_iterations: int = 30
    min_fitness: float = 0.25
    method: str = 'point_to_plane'
    normal_neighbors: int = 8  # target neighbours used to fit each line
    # A direction is degenerate when its information is below this fraction of
    # the best constrained one; the guess is kept there. 0 disables it. A
    # corridor gives ~0.005, a room ~0.3.
    degeneracy_ratio: float = 0.02


@dataclass
class IcpResult:
    pose: Pose2D    # pose of the source robot in the target robot's frame
    fitness: float  # fraction of source points with a correspondence (0..1)
    rmse: float     # RMS distance of those correspondences (inf if none)
    ok: bool        # fitness >= min_fitness
    degenerate: int = 0  # directions (0..3) where the guess was kept


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


def _remap_degenerate(pose, guess, source, target_normals, correspondences, ratio):
    """
    Keep ``guess`` along the directions ICP could not observe.

    The information matrix is built for a small correction ``δ`` applied in the
    target frame (``pose' = δ ⊕ pose``), from the point-to-line residuals of
    the final correspondences. Its units mix m and rad; with points metres away
    the rotation is well scaled against the translations.
    """
    if ratio <= 0.0 or len(correspondences) < 3:
        return pose, 0
    T = to_matrix(pose)
    p = source[correspondences[:, 0]] @ T[:2, :2].T + T[:2, 3]   # matched source points, target frame
    n = target_normals[correspondences[:, 1]]
    J = np.column_stack((n[:, 0], n[:, 1], n[:, 1] * p[:, 0] - n[:, 0] * p[:, 1]))
    eigvals, eigvecs = np.linalg.eigh(J.T @ J)
    weak = eigvecs[:, eigvals < ratio * eigvals[-1]]
    if weak.shape[1] == 0:
        return pose, 0
    delta = np.array(compose(guess, inverse(pose)))               # guess = delta ⊕ pose
    return compose(tuple(weak @ (weak.T @ delta)), pose), weak.shape[1]


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
    _set_line_normals(target_pcd, params.normal_neighbors)
    if params.method == 'point_to_plane':
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
    pose, degenerate = _remap_degenerate(
        from_matrix(reg.transformation), guess, source,
        np.asarray(target_pcd.normals)[:, :2], np.asarray(reg.correspondence_set), params.degeneracy_ratio)
    return IcpResult(pose, fitness, rmse, fitness >= params.min_fitness, degenerate)
