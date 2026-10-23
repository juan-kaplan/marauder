"""
Teach and Repeat map on disk: a chain of keyframes with the scan of each one.

Layout of ``map_dir`` (see ``docs/tp1/arquitectura.md``):

* ``map.json``: the metadata (``robot_frame``, ``mode``, ``tau_d``, ...) plus
  one entry per keyframe, without the points.
* ``scans.npz``: ``kf_0000``, ``kf_0001``, ... → ``(N, 2)`` points in
  ``robot_frame``.

The Repeat follows the chain of ``edge_from_prev``; ``pose_odom`` is only for
plots. Keyframe 0 has no previous keyframe: its edge is the identity and
``edge_icp_ok`` is False.

No ROS imports, so it can be used and tested anywhere.
"""

import json
import os
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

import numpy as np

from rota_common.se2 import IDENTITY, Pose2D, compose

MAP_FILE = 'map.json'
SCANS_FILE = 'scans.npz'


def scan_key(kf_id: int) -> str:
    """Key of a keyframe's points inside ``scans.npz``."""
    return f'kf_{kf_id:04d}'


def _pose(p: Sequence[float]) -> Pose2D:
    x, y, t = p
    return (float(x), float(y), float(t))


@dataclass
class Keyframe:
    id: int
    stamp: float                # seconds, stamp of the reference that created it
    pose_odom: Pose2D           # pose in odom when it was taught (plots only)
    edge_from_prev: Pose2D = IDENTITY  # pose of this keyframe seen from the previous one
    edge_fitness: float = 0.0
    edge_icp_ok: bool = False
    points: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))  # (N, 2) in robot_frame

    def __post_init__(self):
        self.id = int(self.id)
        self.stamp = float(self.stamp)
        self.pose_odom = _pose(self.pose_odom)
        self.edge_from_prev = _pose(self.edge_from_prev)
        self.edge_fitness = float(self.edge_fitness)
        self.edge_icp_ok = bool(self.edge_icp_ok)
        self.points = np.asarray(self.points, dtype=np.float64).reshape(-1, 2)

    def to_json(self) -> dict:
        return {
            'id': self.id,
            'stamp': self.stamp,
            'pose_odom': list(self.pose_odom),
            'edge_from_prev': list(self.edge_from_prev),
            'edge_fitness': self.edge_fitness,
            'edge_icp_ok': self.edge_icp_ok,
            'num_points': int(len(self.points)),
        }


def _check_ids(keyframes: Sequence[Keyframe]):
    for i, kf in enumerate(keyframes):
        if kf.id != i:
            raise ValueError(f'keyframe ids must be 0, 1, 2, ...: position {i} has id {kf.id}')


def save(map_dir: str, keyframes: Sequence[Keyframe], meta: Dict) -> None:
    """
    Write ``map.json`` and ``scans.npz`` into ``map_dir`` (created if missing).

    Both files are written to a temporary name and then renamed, scans first,
    so an interrupted save never leaves a ``map.json`` without its scans.
    """
    if 'keyframes' in meta:
        raise ValueError("meta must not contain 'keyframes'")
    _check_ids(keyframes)
    os.makedirs(map_dir, exist_ok=True)

    scans_path = os.path.join(map_dir, SCANS_FILE)
    with open(scans_path + '.tmp', 'wb') as f:
        np.savez_compressed(f, **{scan_key(kf.id): kf.points for kf in keyframes})
    os.replace(scans_path + '.tmp', scans_path)

    map_path = os.path.join(map_dir, MAP_FILE)
    doc = dict(meta)
    doc['keyframes'] = [kf.to_json() for kf in keyframes]
    with open(map_path + '.tmp', 'w') as f:
        json.dump(doc, f, indent=2)
    os.replace(map_path + '.tmp', map_path)


def load(map_dir: str) -> Tuple[Dict, List[Keyframe]]:
    """Read a map written by :func:`save`. Returns ``(meta, keyframes)``."""
    map_path = os.path.join(map_dir, MAP_FILE)
    scans_path = os.path.join(map_dir, SCANS_FILE)
    for path in (map_path, scans_path):
        if not os.path.exists(path):
            raise FileNotFoundError(f"map file '{path}' not found")

    with open(map_path) as f:
        doc = json.load(f)
    entries = doc.pop('keyframes', [])

    keyframes = []
    with np.load(scans_path) as scans:
        for entry in entries:
            key = scan_key(entry['id'])
            if key not in scans:
                raise ValueError(f"'{scans_path}' has no '{key}'")
            kf = Keyframe(
                id=entry['id'],
                stamp=entry['stamp'],
                pose_odom=entry['pose_odom'],
                edge_from_prev=entry['edge_from_prev'],
                edge_fitness=entry['edge_fitness'],
                edge_icp_ok=entry['edge_icp_ok'],
                points=scans[key],
            )
            if len(kf.points) != entry.get('num_points', len(kf.points)):
                raise ValueError(f"'{key}' has {len(kf.points)} points, "
                                 f"map.json says {entry['num_points']}")
            keyframes.append(kf)
    _check_ids(keyframes)
    return doc, keyframes


def chain_poses(keyframes: Sequence[Keyframe]) -> List[Pose2D]:
    """Pose of every keyframe in the frame of keyframe 0, following ``edge_from_prev``."""
    poses = []
    pose = IDENTITY
    for i, kf in enumerate(keyframes):
        if i > 0:
            pose = compose(pose, kf.edge_from_prev)
        poses.append(pose)
    return poses
