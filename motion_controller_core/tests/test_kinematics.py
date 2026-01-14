import pytest
import numpy as np
import pinocchio as pin
from motion_controller_core.kinematics.pinocchio_solver import PinocchioSolver

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
    s = PinocchioSolver()
    success = s.init(URDF_CONTENT, "base_link", "end_effector")
    assert success, "Solver initialization failed"
    return s

def test_initialization(solver):
    assert solver.model.nq == 2
    assert solver.model.nv == 2
    assert solver.tip_frame_id is not None

def test_forward_kinematics(solver):
    # Zero configuration: Straight line along X
    # Link1 len = 1, Link2 len = 1 => Total X = 2
    q = np.zeros(2)
    success, pose = solver.solve_fk(q)
    
    assert success
    expected_pos = np.array([2.0, 0.0, 0.0])
    np.testing.assert_allclose(pose.translation, expected_pos, atol=1e-5)
    
    # 90 degrees at joint 1
    # Link 1 along Y, Link 2 along Y (relative), so Global Link 2 along Y.
    # Pos: (0, 2, 0)
    q = np.array([np.pi/2, 0.0])
    success, pose = solver.solve_fk(q)
    assert success
    np.testing.assert_allclose(pose.translation, np.array([0.0, 2.0, 0.0]), atol=1e-5)

def test_inverse_kinematics(solver):
    # Target: (0, 2, 0) -> q should be (pi/2, 0)
    target_pose = pin.SE3.Identity()
    target_pose.translation = np.array([0.0, 2.0, 0.0])
    # Rotation: 90 deg around Z
    target_pose.rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    
    q_init = np.array([0.1, 0.1])
    success, q_out = solver.solve_ik(target_pose, q_init)
    
    assert success
    np.testing.assert_allclose(q_out, np.array([np.pi/2, 0.0]), atol=1e-3)

def test_inverse_kinematics_unreachable(solver):
    # Target: (10, 0, 0) -> Unreachable (max reach is 2)
    target_pose = pin.SE3.Identity()
    target_pose.translation = np.array([10.0, 0.0, 0.0])
    
    q_init = np.zeros(2)
    success, q_out = solver.solve_ik(target_pose, q_init)
    
    # Simple CLIK might not "fail" via boolean but converge to closest.
    # Our implementation returns False if error norm < eps is not met.
    # So it should return False.
    assert not success

if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__]))

