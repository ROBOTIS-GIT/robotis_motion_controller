import os
import sys

# Make the local `retargeting` package importable
sys.path.insert(0, os.path.dirname(__file__))

import mujoco
import numpy as np
import tyro
from mujoco import viewer as mj_viewer

from retargeting.seq_retarget import ROBOTISHandRetargeter
from xrobotoolkit_teleop.common.xr_client import XrClient
from xrobotoolkit_teleop.utils.dex_hand_utils import (
    build_palm_frame_from_landmarks,
    pico_hand_state_to_mediapipe,
)

# ===== MANO convention → Robot URDF base frame =====
#   MANO:  X = backward,  Y = dorsal,  Z = radial
#   Robot: X = palmar,    Y = radial,  Z = forward
#
#   Robot_X = -MANO_Y  (palmar = opposite of dorsal)
#   Robot_Y =  MANO_Z  (radial = same)
#   Robot_Z = -MANO_X  (forward = opposite of backward)
#
#   M[i,j] = how much MANO component i contributes to Robot component j
#   row 0 (MANO_X=backward):  → -Robot_Z (forward)
#   row 1 (MANO_Y=dorsal):    → -Robot_X (palmar)
#   row 2 (MANO_Z=radial):    →  Robot_Y (radial)

_R_MANO_TO_ROBOT = np.array([
    [ 0,  0, -1],
    [-1,  0,  0],
    [ 0,  1,  0],
], dtype=np.float64)

PRE_CALIBRATION = [0.122617, 0.169113, 0.180041, 0.164834, 0.145511]

# ===== Coordinate transform =====

def _to_palm_local(mediapipe_pose: np.ndarray, hand_side: str = "right",) -> tuple[np.ndarray, np.ndarray]:
    palm_frame = build_palm_frame_from_landmarks(mediapipe_pose, hand_side=hand_side)
    return mediapipe_pose @ palm_frame, palm_frame

# ===== Actuator mapping =====

def build_qpos_to_actuator_map(mj_model, dof_joint_names: list[str]) -> dict[int, int]:
    """Map retargeter qpos indices to MuJoCo actuator IDs."""
    mapping: dict[int, int] = {}
    for qpos_idx, joint_name in enumerate(dof_joint_names):
        act_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)
        if act_id >= 0:
            mapping[qpos_idx] = act_id
        else:
            print(f"[Warning] Joint '{joint_name}' has no matching actuator in the MuJoCo model.")
    return mapping


def main(
    xml_path: str = os.path.join(os.path.dirname(__file__), "../robotis_ffw/scene_ffw_sh5.xml"),
    urdf_path: str = os.path.join(os.path.dirname(__file__), "../hx5_d20_right.urdf"),
    hand_type: str = "right",
):

    # ===== MuJoCo model ===== 
    mj_model = mujoco.MjModel.from_xml_path(xml_path)
    mj_data = mujoco.MjData(mj_model)

    # ===== Retargeter =====
    retargeter = ROBOTISHandRetargeter(path_to_urdf=urdf_path, hand_side=hand_type)
    print(f"[Retargeter] DOF joint names: {retargeter.robot.dof_joint_names}")

    retargeter.set_pre_calibration(PRE_CALIBRATION)

    qpos_to_actuator = build_qpos_to_actuator_map(mj_model, retargeter.robot.dof_joint_names)
    print(f"[Retargeter] Mapped {len(qpos_to_actuator)} joints to actuators.")
    dof_names = retargeter.robot.dof_joint_names
    for qpos_idx, act_id in qpos_to_actuator.items():
        print(f"  qpos[{qpos_idx:2d}] {dof_names[qpos_idx]:20s} (act_id={act_id})")

    # ===== XR client =====
    xr_client = XrClient()

    # ===== Main loop =====
    with mj_viewer.launch_passive(mj_model, mj_data) as viewer:
        viewer.cam.azimuth  = 180
        viewer.cam.elevation = -20
        viewer.cam.distance  = 1.5
        viewer.cam.lookat    = [0.0, 0.0, 0.5]

        while viewer.is_running():
            # 1. Receive PICO data to MediaPipe (21, 3) wrist-centred world frame
            hand_state = xr_client.get_hand_tracking_state(hand_type)
            if hand_state is not None:
                hand_state_np = np.array(hand_state)   # expected (27, 7)

                if hand_state_np.ndim == 2 and hand_state_np.shape[0] >= 26:
                    try:
                        mediapipe_pose = pico_hand_state_to_mediapipe(hand_state_np)  # (21, 3)
                        # NOTE: 2. Convert to canonical palm-local frame (MANO convention)
                        #    This separates two concerns: geometric normalisation (making finger
                        #    positions relative to the palm, independent of wrist position and world
                        #    orientation) and robot-axis alignment.  The robot-specific part is
                        #    isolated to the single matrix _R_MANO_TO_ROBOT in step 3, so adapting
                        #    to a different robot hand only requires updating that matrix.
                        mediapipe_local, _ = _to_palm_local(mediapipe_pose, hand_side=hand_type)

                        # 3. Transform from MANO convention to robot URDF frame,
                        #    then retarget to robot joint positions.
                        robot_local = mediapipe_local @ _R_MANO_TO_ROBOT
                        result = retargeter.retarget(robot_local)

                        # 4. Write robot_qpos into mj_data.ctrl.
                        for qpos_idx, act_id in qpos_to_actuator.items():
                            mj_data.ctrl[act_id] = result.robot_qpos[qpos_idx]

                    except ValueError as e:
                        print(f"[Warning] Retargeting skipped: {e}")

            mujoco.mj_step(mj_model, mj_data)
            viewer.sync()


if __name__ == "__main__":
    tyro.cli(main)
