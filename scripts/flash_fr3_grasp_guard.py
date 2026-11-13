"""Demonstration grasp geometry gate; no cube detection or collision planning."""
import hashlib

import numpy as np

from flash_fr3_kinematics import UrdfKinematics


class GraspGeometryGate:
    def __init__(self, urdf, grasp_q):
        self.fk = UrdfKinematics(urdf)
        self.q = np.asarray(grasp_q, dtype=float)
        if self.q.ndim != 2 or self.q.shape[1] != 7 or not len(self.q) or not np.isfinite(self.q).all():
            raise ValueError('缺少有限的示教抓取关节姿态')
        self.transforms = self.fk.transforms(self.q)
        self.positions = self.transforms[:, :3, 3]
        self.z_lower = float(self.positions[:, 2].min()-.01)
        self.z_upper = float(self.positions[:, 2].max()+.01)
        self.description = {
            'format': 'flash_fr3_grasp_geometry_gate_v1',
            'base_frame': self.fk.base, 'tip_frame': self.fk.tip,
            'urdf_sha256': hashlib.sha256(urdf.read_bytes()).hexdigest(),
            'training_grasp_events': len(self.q),
            'max_position_error_m': .02, 'max_rotation_error_deg': 10.,
            'max_joint_error_rad': .06,
            'allowed_flange_z_m': [self.z_lower, self.z_upper],
            'scope': 'same demonstration event must satisfy position, rotation and joint checks; '
                     'flange geometry only, not calibrated fingertips, cube detection or collision checking'}

    def check(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError('抓取几何检查需要有限的七关节状态')
        transform = self.fk.transforms(q[None])[0]
        xyz = transform[:3, 3]
        distances = np.linalg.norm(self.positions-xyz, axis=1)
        # trace(R_event.T @ R_actual) is the Frobenius inner product.
        trace = np.sum(self.transforms[:, :3, :3]*transform[:3, :3], axis=(1, 2))
        angles = np.rad2deg(np.arccos(np.clip((trace-1)/2, -1., 1.)))
        joint_gaps = abs(self.q-q).max(axis=1)
        compatible = ((distances <= .02) & (angles <= 10.) & (joint_gaps <= .06))
        height_ok = self.z_lower <= xyz[2] <= self.z_upper
        nearest = int(np.argmin(distances))
        accepted = bool(height_ok and compatible.any())
        reasons = []
        if not height_ok:
            reasons.append('法兰高度超出示教抓取高度带')
        if not compatible.any():
            reasons.append('没有同时满足位置、朝向和关节角条件的示教抓取姿态')
        return {'accepted': accepted, 'reasons': reasons,
                'flange_position_m': xyz.tolist(),
                'allowed_flange_z_m': [self.z_lower, self.z_upper],
                'compatible_events': int(compatible.sum()),
                'nearest_position_event_index': nearest,
                'nearest_position_distance_m': float(distances[nearest]),
                'nearest_position_event_z_m': float(self.positions[nearest, 2]),
                'nearest_position_rotation_error_deg': float(angles[nearest]),
                'nearest_position_joint_gap_rad': float(joint_gaps[nearest]),
                'nearest_joint_gap_rad': float(joint_gaps.min())}
