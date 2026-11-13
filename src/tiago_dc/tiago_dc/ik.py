"""Damped-least-squares IK for one TIAGo Pro arm on the full URDF model (pinocchio).

Only the arm's own joints move; every other joint (torso, head, other arm) is held at its last
measured value, so the solution stays valid for whatever torso height the robot is at.
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin


class ArmIK:
    def __init__(self, urdf_xml: str, arm_joints: list[str], tcp_frame: str, world_frame: str,
                 limit_margin: float = 0.05, damping: float = 0.05):
        self.model = pin.buildModelFromXML(urdf_xml)
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        self.tcp_id = self._frame_id(tcp_frame)
        self.world_id = self._frame_id(world_frame)
        joint_ids = []
        for name in arm_joints:
            if not self.model.existJointName(name):
                raise ValueError(f"joint {name} not in robot_description")
            joint_ids.append(self.model.getJointId(name))
        self.q_idx = np.array([self.model.joints[j].idx_q for j in joint_ids])
        self.v_idx = np.array([self.model.joints[j].idx_v for j in joint_ids])
        self.lower = self.model.lowerPositionLimit[self.q_idx] + limit_margin
        self.upper = self.model.upperPositionLimit[self.q_idx] - limit_margin
        self.damping = damping

    def _frame_id(self, name: str) -> int:
        if not self.model.existFrame(name):
            raise ValueError(f"frame {name} not in robot_description")
        return self.model.getFrameId(name)

    def set_joint_positions(self, positions: dict[str, float]):
        """Update the held configuration from a name -> position map (e.g. /joint_states)."""
        for name, value in positions.items():
            if not self.model.existJointName(name):
                continue
            joint = self.model.joints[self.model.getJointId(name)]
            if joint.nq == 1:
                self.q[joint.idx_q] = value
            elif joint.nq == 2:  # continuous joint stored as (cos, sin)
                self.q[joint.idx_q:joint.idx_q + 2] = (np.cos(value), np.sin(value))

    def clamp(self, q_arm: np.ndarray) -> np.ndarray:
        return np.clip(q_arm, self.lower, self.upper)

    def fk(self, q_arm: np.ndarray) -> pin.SE3:
        """TCP pose in the world frame for the given arm joints."""
        q = self.q.copy()
        q[self.q_idx] = q_arm
        pin.framesForwardKinematics(self.model, self.data, q)
        return self.data.oMf[self.world_id].actInv(self.data.oMf[self.tcp_id])

    def solve(self, target: pin.SE3, q_seed: np.ndarray, iterations: int = 8,
              max_step: float | None = None) -> tuple[np.ndarray, float]:
        """Move from q_seed toward the world-frame TCP target.

        Returns the new arm joints and the remaining pose error (norm of the local twist). With
        max_step, no joint moves farther than that from q_seed, so callers get a joint-velocity
        limit by passing max_joint_speed * dt.
        """
        q = self.q.copy()
        q_arm = self.clamp(np.asarray(q_seed, dtype=float))
        error = np.inf
        for _ in range(iterations):
            q[self.q_idx] = q_arm
            pin.framesForwardKinematics(self.model, self.data, q)
            current = self.data.oMf[self.world_id].actInv(self.data.oMf[self.tcp_id])
            twist = pin.log6(current.actInv(target)).vector
            error = float(np.linalg.norm(twist))
            if error < 1e-4:
                break
            jacobian = pin.computeFrameJacobian(self.model, self.data, q, self.tcp_id, pin.LOCAL)
            jacobian = jacobian[:, self.v_idx]
            gram = jacobian @ jacobian.T + (self.damping ** 2) * np.eye(6)
            q_arm = self.clamp(q_arm + jacobian.T @ np.linalg.solve(gram, twist))
            if max_step is not None:
                q_arm = q_seed + np.clip(q_arm - q_seed, -max_step, max_step)
        return q_arm, error
