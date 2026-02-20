from typing import List

import numpy as np
import numpy.typing as npt
import pinocchio as pin
from autograd.extend import primitive, defvjp


# def _skew(v):
#     """Convert 3D vector to skew-symmetric matrix."""
#     return np.array([
#         [0, -v[2], v[1]],
#         [v[2], 0, -v[0]],
#         [-v[1], v[0], 0]
#     ])

class RobotWrapper:
    """Pinocchio robot wrapper with Autograd-compatible forward kinematics."""

    def __init__(self, urdf_path: str):
        # Create robot model and data
        self.model: pin.Model = pin.buildModelFromUrdf(urdf_path)
        self.data: pin.Data = self.model.createData()

        # print(self.model)

        if self.model.nv != self.model.nq:
            raise NotImplementedError("Cannot handle robot with special joint.")

        # Cache for differentiable FK primitives
        self._fk_primitives = {}

    @property
    def dof_joint_names(self) -> List[str]:
        """Return names of joints with DOF > 0."""
        nqs = self.model.nqs
        return [name for i, name in enumerate(self.model.names) if nqs[i] > 0]

    @property
    def joint_limits(self):
        """Return joint limits as (lower, upper) pairs."""
        lower = self.model.lowerPositionLimit
        upper = self.model.upperPositionLimit
        return np.stack([lower, upper], axis=1)

    def get_link_index(self, name: str) -> int:
        """Get frame index by name."""
        return self.model.getFrameId(name, pin.BODY)

    # -------------------------------------------------------------------------- #
    # Standard Kinematics Methods
    # -------------------------------------------------------------------------- #
    def compute_forward_kinematics(self, qpos: npt.NDArray):
        """Compute forward kinematics for all links."""
        pin.forwardKinematics(self.model, self.data, qpos)

    def get_link_pose(self, link_id: int) -> npt.NDArray:
        """Get link pose as 4x4 homogeneous matrix."""
        pose: pin.SE3 = pin.updateFramePlacement(self.model, self.data, link_id)
        return pose.homogeneous

    def compute_single_link_local_jacobian(self, qpos, link_id: int) -> npt.NDArray:
        """Compute Jacobian for a single link."""
        J = pin.computeFrameJacobian(self.model, self.data, qpos, link_id)
        return J

    # -------------------------------------------------------------------------- #
    # Differentiable Kinematics Methods (Autograd-compatible)
    # -------------------------------------------------------------------------- #
    def differentiable_fk(self, qpos, link_indices: List[int]):
        """Autograd-compatible FK. Returns flattened link positions (num_links * 3,)."""
        # Create primitive for this link combination if not cached
        key = tuple(sorted(link_indices))
        if key not in self._fk_primitives:
            self._fk_primitives[key] = self._make_fk_primitive(link_indices)

        return self._fk_primitives[key](qpos)

    def differentiable_fk_rot(self, qpos, link_indices: List[int]):
        """Autograd-compatible FK returning rotation matrices.

        Returns:
            rotations: (num_links, 3, 3) array of rotation matrices.
        """
        # Create primitive for this link combination if not cached
        key = "rot_" + str(sorted(link_indices))
        if key not in self._fk_primitives:
            self._fk_primitives[key] = self._make_fk_rot_primitive(link_indices)

        return self._fk_primitives[key](qpos)

    def _make_fk_primitive(self, link_indices: List[int]):
        """Create Autograd primitive for specific link indices."""
        robot = self

        @primitive
        def fk(qpos):
            qpos = np.asarray(qpos, dtype=np.float64)
            robot.compute_forward_kinematics(qpos)
            positions = np.array([
                robot.get_link_pose(idx)[:3, 3] for idx in link_indices
            ], dtype=np.float64)
            return positions.flatten()

        def fk_vjp(ans, qpos):
            qpos = np.asarray(qpos, dtype=np.float64)

            def vjp_fn(g):
                # g: upstream gradient (num_links * 3,)
                g = g.reshape(-1, 3)

                # Recompute FK for Jacobians
                robot.compute_forward_kinematics(qpos)

                # Compute and stack Jacobians
                jacobians = []
                for i, idx in enumerate(link_indices):
                    J_local = robot.compute_single_link_local_jacobian(qpos, idx)[:3, ...]
                    R = robot.get_link_pose(idx)[:3, :3]
                    jacobians.append(R @ J_local)

                J = np.stack(jacobians, axis=0)  # (num_links, 3, num_joints)

                # Chain rule: grad_qpos = sum over links of (g @ J)
                return np.einsum('li,lij->j', g, J)

            return vjp_fn

        defvjp(fk, fk_vjp)
        return fk

    def _make_fk_rot_primitive(self, link_indices: List[int]):
        """Create Autograd primitive for rotation FK."""
        robot = self

        @primitive
        def fk_rot(qpos):
            qpos = np.asarray(qpos, dtype=np.float64)
            robot.compute_forward_kinematics(qpos)
            rotations = np.array([
                robot.get_link_pose(idx)[:3, :3] for idx in link_indices
            ], dtype=np.float64)
            return rotations

        def fk_rot_vjp(ans, qpos):
            qpos = np.asarray(qpos, dtype=np.float64)

            def vjp_fn(g):
                # g: upstream gradient (num_links, 3, 3)
                # We need to compute sum( g * dR/dq )

                robot.compute_forward_kinematics(qpos)

                N_joints = len(qpos)
                
                grad_q = np.zeros(N_joints)
                
                for i, idx in enumerate(link_indices):
                    # Get current Rotation
                    R = robot.get_link_pose(idx)[:3, :3]
                    
                    # Get Jacobian (6, N_joints) - angular part is bottom 3 rows
                    J_local = robot.compute_single_link_local_jacobian(qpos, idx)
                    J_ang_local = J_local[3:, :] # (3, N_joints)
                    
                    # Transform to world frame angular Jacobian: J_w = R @ J_local
                    # Shape (3, N_joints)
                    J_ang_world = R @ J_ang_local 
                    
                    # Upstream gradient for this link (3, 3)
                    g_link = g[i]
                    
                    K = R @ g_link.T
                    p = np.array([
                        K[2, 1] - K[1, 2],
                        K[0, 2] - K[2, 0],
                        K[1, 0] - K[0, 1]
                    ])

                    grad_q += J_ang_world.T @ p

                return grad_q

            return vjp_fn

        defvjp(fk_rot, fk_rot_vjp)
        return fk_rot
