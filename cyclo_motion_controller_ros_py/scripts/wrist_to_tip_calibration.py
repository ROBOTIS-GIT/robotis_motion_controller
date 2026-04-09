"""
Usage:
    python wrist_to_tip_calibration.py
    python wrist_to_tip_calibration.py --hand_type left
Controls:
    s  - start / stop sampling
    q  - quit and print the final calibration array
"""

import os
import sys
import threading

sys.path.insert(0, os.path.dirname(__file__))

import numpy as np

from xrobotoolkit_teleop.common.xr_client import XrClient
from xrobotoolkit_teleop.utils.dex_hand_utils import (build_palm_frame_from_landmarks, pico_hand_state_to_mediapipe,)

MP_WRIST_IDX = 0
MP_FINGER_TIP_INDICES = [4, 8, 12, 16, 20]
FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

def _compute_human_finger_lengths(mediapipe_pose: np.ndarray) -> np.ndarray:
    wrist_pos = mediapipe_pose[MP_WRIST_IDX]
    tip_positions = mediapipe_pose[MP_FINGER_TIP_INDICES]
    return np.linalg.norm(tip_positions - wrist_pos, axis=1).astype(np.float32)

def _keyboard_listener(state: dict) -> None:
    import termios
    import tty

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while not state["quit"]:
            ch = sys.stdin.read(1)
            if ch == "s":
                state["sampling"] = not state["sampling"]
                label = "STARTED" if state["sampling"] else "PAUSED"
                print(f"\r[Calibration] Sampling {label}                    ")
            elif ch == "q":
                state["quit"] = True
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(hand_type: str = "right") -> None:
    xr_client = XrClient()

    state = {"sampling": False, "quit": False}
    kb_thread = threading.Thread(target=_keyboard_listener, args=(state,), daemon=True)
    kb_thread.start()

    print("=" * 60)
    print("  ROBOTIS Hand Finger-Length Calibration")
    print("  Press  s  to start/pause sampling")
    print("  Press  q  to quit and print the result")
    print("=" * 60)

    accumulated: list[np.ndarray] = [] 

    try:
        while not state["quit"]:
            hand_state = xr_client.get_hand_tracking_state(hand_type)
            if hand_state is None:
                continue
            hand_state_np = np.array(hand_state)
            if hand_state_np.ndim != 2 or hand_state_np.shape[0] < 26:
                continue
            try:
                mediapipe_pose = pico_hand_state_to_mediapipe(hand_state_np)  # (21, 3)
            except ValueError:
                continue

            finger_lengths = _compute_human_finger_lengths(mediapipe_pose)  # (5,)

            if state["sampling"]:
                accumulated.append(finger_lengths)
                mean_lengths = np.mean(accumulated, axis=0)
                n = len(accumulated)

                parts = "  ".join(f"{name}={length:.4f}"for name, length in zip(FINGER_NAMES, mean_lengths))
                print(f"\r[n={n:5d}]  {parts}", end="", flush=True)

    except KeyboardInterrupt:
        pass

    print()

    if not accumulated:
        print("[Calibration] No samples collected. Exiting.")
        return

    final_lengths = np.mean(accumulated, axis=0)

    print()
    print("=" * 60)
    print(f"  Samples collected : {len(accumulated)}")
    print("  Mean finger lengths (wrist → tip, palm-local frame):")
    for name, length in zip(FINGER_NAMES, final_lengths): print(f"    {name:8s}: {length:.6f} m")
    print()
    print("  Copy the line below PRE_CALIBRATION  --human_finger_lengths:")
    arr_str = ", ".join(f"{v:.6f}" for v in final_lengths)
    print(f"    [{arr_str}]")
    print("=" * 60)

if __name__ == "__main__":
    import tyro
    tyro.cli(main)
