"""
Teach and Repeat map, saved in map_dir.

map.json has the metadata and the keyframes, scans.npz their points
(kf_0000, kf_0001, ... -> (N, 2) in robot_frame). Keyframe 0 has an identity
edge_from_prev.
"""

import json
import os
from dataclasses import dataclass, field

import numpy as np

from rota_common.se2 import IDENTITY, compose


@dataclass
class Keyframe:
    id: int
    stamp: float
    pose_odom: tuple             # only for plots
    edge_from_prev: tuple = IDENTITY
    edge_fitness: float = 0.0
    edge_icp_ok: bool = False
    points: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))


def save(map_dir, keyframes, meta):
    os.makedirs(map_dir, exist_ok=True)
    np.savez_compressed(os.path.join(map_dir, 'scans.npz'),
                        **{f'kf_{kf.id:04d}': kf.points for kf in keyframes})
    doc = dict(meta)
    doc['keyframes'] = [{
        'id': kf.id, 'stamp': kf.stamp, 'pose_odom': list(kf.pose_odom),
        'edge_from_prev': list(kf.edge_from_prev), 'edge_fitness': float(kf.edge_fitness),
        'edge_icp_ok': bool(kf.edge_icp_ok), 'num_points': len(kf.points),
    } for kf in keyframes]
    with open(os.path.join(map_dir, 'map.json'), 'w') as f:
        json.dump(doc, f, indent=2)


def load(map_dir):
    """Return (meta, keyframes)."""
    with open(os.path.join(map_dir, 'map.json')) as f:
        meta = json.load(f)
    keyframes = []
    with np.load(os.path.join(map_dir, 'scans.npz')) as scans:
        for e in meta.pop('keyframes'):
            keyframes.append(Keyframe(e['id'], e['stamp'], tuple(e['pose_odom']),
                                      tuple(e['edge_from_prev']), e['edge_fitness'],
                                      e['edge_icp_ok'], scans[f"kf_{e['id']:04d}"]))
    return meta, keyframes


def chain_poses(keyframes):
    """Pose of each keyframe in the frame of keyframe 0."""
    poses = []
    for i, kf in enumerate(keyframes):
        poses.append(IDENTITY if i == 0 else compose(poses[-1], kf.edge_from_prev))
    return poses
