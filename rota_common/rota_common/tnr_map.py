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


def scan_key(kf_id):
    return f'kf_{kf_id:04d}'


def as_pose(p):
    return tuple(float(v) for v in p)


@dataclass
class Keyframe:
    id: int
    stamp: float
    pose_odom: tuple             # only for plots
    edge_from_prev: tuple = IDENTITY
    edge_fitness: float = 0.0
    edge_icp_ok: bool = False
    points: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))

    def __post_init__(self):
        self.id = int(self.id)
        self.stamp = float(self.stamp)
        self.pose_odom = as_pose(self.pose_odom)
        self.edge_from_prev = as_pose(self.edge_from_prev)
        self.edge_fitness = float(self.edge_fitness)
        self.edge_icp_ok = bool(self.edge_icp_ok)
        self.points = np.asarray(self.points, dtype=np.float64).reshape(-1, 2)


def check_ids(keyframes):
    for i, kf in enumerate(keyframes):
        if kf.id != i:
            raise ValueError(f"keyframe ids must be 0, 1, 2, ...: position {i} has id {kf.id}")


def save(map_dir, keyframes, meta):
    if 'keyframes' in meta:
        raise ValueError("meta must not contain 'keyframes'")
    check_ids(keyframes)
    os.makedirs(map_dir, exist_ok=True)
    # Scans first, and each file through a rename, so a map.json never lacks its scans.
    scans_path = os.path.join(map_dir, 'scans.npz')
    with open(scans_path + '.tmp', 'wb') as f:
        np.savez_compressed(f, **{scan_key(kf.id): kf.points for kf in keyframes})
    os.replace(scans_path + '.tmp', scans_path)

    doc = dict(meta)
    doc['keyframes'] = [{
        'id': kf.id, 'stamp': kf.stamp, 'pose_odom': list(kf.pose_odom),
        'edge_from_prev': list(kf.edge_from_prev), 'edge_fitness': kf.edge_fitness,
        'edge_icp_ok': kf.edge_icp_ok, 'num_points': len(kf.points),
    } for kf in keyframes]
    map_path = os.path.join(map_dir, 'map.json')
    with open(map_path + '.tmp', 'w') as f:
        json.dump(doc, f, indent=2)
    os.replace(map_path + '.tmp', map_path)


def load(map_dir):
    """Return (meta, keyframes)."""
    map_path = os.path.join(map_dir, 'map.json')
    scans_path = os.path.join(map_dir, 'scans.npz')
    for path in (map_path, scans_path):
        if not os.path.exists(path):
            raise FileNotFoundError(f"map file '{path}' not found")
    with open(map_path) as f:
        meta = json.load(f)
    keyframes = []
    with np.load(scans_path) as scans:
        for e in meta.pop('keyframes', []):
            key = scan_key(e['id'])
            if key not in scans:
                raise ValueError(f"'{scans_path}' has no '{key}'")
            kf = Keyframe(e['id'], e['stamp'], e['pose_odom'], e['edge_from_prev'],
                          e['edge_fitness'], e['edge_icp_ok'], scans[key])
            if len(kf.points) != e.get('num_points', len(kf.points)):
                raise ValueError(f"'{key}' has {len(kf.points)} points, "
                                 f"map.json says {e['num_points']}")
            keyframes.append(kf)
    check_ids(keyframes)
    return meta, keyframes


def chain_poses(keyframes):
    """Pose of each keyframe in the frame of keyframe 0."""
    poses = []
    for i, kf in enumerate(keyframes):
        poses.append(IDENTITY if i == 0 else compose(poses[-1], kf.edge_from_prev))
    return poses
