import pytest
import numpy as np
import os
from motion_controller_core.kinematics.pyroki_solver import PyrokiSolver
from scipy.spatial.transform import Rotation

# Simple 2 DOF Planar Arm URDF
URDF_CONTENT = """
<robot name="test_robot">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="end_effector"/>
  
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  
  <joint name="fixed_ee" type="fixed">
    <parent link="link2"/>
    <child link="end_effector"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""

@pytest.fixture
def solver():
    s = PyrokiSolver()
    success = s.init(URDF_CONTENT, "base_link", ["end_effector"])
    if not success:
        pytest.skip("Pyroki initialization failed")
    return s

def test_initialization(solver):
    assert solver.robot is not None
    assert solver.target_link_indices is not None
    assert len(solver.target_link_indices) == 1

def test_forward_kinematics(solver):
    # Zero config: (2, 0, 0)
    q = np.zeros(2)
    success, poses = solver.solve_fk(q)
    assert success
    assert len(poses) == 1
    
    expected_pos = np.array([2.0, 0.0, 0.0])
    np.testing.assert_allclose(poses[0][:3, 3], expected_pos, atol=1e-4)
    
    # 90 deg at joint 1: (0, 2, 0)
    q = np.array([np.pi/2, 0.0])
    success, poses = solver.solve_fk(q)
    assert success
    np.testing.assert_allclose(poses[0][:3, 3], np.array([0.0, 2.0, 0.0]), atol=1e-4)

def test_inverse_kinematics(solver):
    # Target: (0, 2, 0) -> q should be (pi/2, 0)
    target_pose = np.eye(4)
    target_pose[:3, 3] = np.array([0.0, 2.0, 0.0])
    # Rotation: 90 deg around Z
    R = Rotation.from_euler('z', 90, degrees=True).as_matrix()
    target_pose[:3, :3] = R
    
    q_init = np.array([0.1, 0.1])
    success, q_out = solver.solve_ik([target_pose], q_init)
    
    assert success
    # Check if result is close to expected
    np.testing.assert_allclose(q_out, np.array([np.pi/2, 0.0]), atol=0.05)

if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__]))
