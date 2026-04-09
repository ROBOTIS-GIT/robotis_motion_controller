# Teleoperation Example — MuJoCo + PICO VR → HX5-D20

This example shows how to use the `retargeting` package from this repository
in a standalone, ROS-free workspace to teleoperate the HX5-D20 dexterous hand
inside a MuJoCo simulation using a PICO VR headset.

## Table of Contents

1. [Overview](#overview)
2. [Configuration](#configuration)
3. [Running](#running)
4. [Pipeline Walkthrough](#pipeline-walkthrough)


---

## Overview

```
PICO VR (XrClient)
      │  raw hand-joint state  (≥26 × 7)
      ▼
pico_hand_state_to_mediapipe()        xrobotoolkit_teleop
      │  MediaPipe landmarks  (21 × 3)  wrist-centred world frame
      ▼
_to_palm_local()                      geometric normalisation
      │  finger positions relative to palm  (MANO convention)
      ▼
@ _R_MANO_TO_ROBOT                    robot-specific axis rotation
      │  landmarks in robot URDF base frame
      ▼
ROBOTISHandRetargeter.retarget()      retargeting (this repo)
      │  robot joint positions  (qpos)
      ▼
mj_data.ctrl                          MuJoCo passive viewer
```

The `retargeting/` package is the **only** dependency on this repository.
Everything else (MuJoCo, PICO SDK, MuJoCo scene XML) is external.

---

## Configuration

**`PRE_CALIBRATION`**

```python
PRE_CALIBRATION = [0.122617, 0.169113, 0.180041, 0.164834, 0.145511]
```

Five wrist-to-fingertip distances in metres, ordered thumb → pinky, measured
on the operator's hand. These are applied at startup so that the retargeter
does not rely on a potentially noisy first frame to auto-calibrate finger
scaling.

**`_R_MANO_TO_ROBOT`**

```python
_R_MANO_TO_ROBOT = np.array([
    [ 0,  0, -1],
    [-1,  0,  0],
    [ 0,  1,  0],
], dtype=np.float64)
```

3 × 3 rotation matrix mapping MANO palm-local axes to the HX5-D20 URDF base
frame. If you target a different robot hand, this is the only matrix to
update. See [Pipeline Walkthrough](#pipeline-walkthrough) for the axis
conventions.

---

## Running

```bash
cd examples/teleoperation_mujoco_pico

# Default paths
python main_control.py

# Custom paths
python main_control.py \
  --xml-path /path/to/scene.xml \
  --urdf-path /path/to/hx5_d20_right.urdf \
  --hand-type right
```
```bash
python main_control.py --help
```

---

## Pipeline Walkthrough

### Step 1 — PICO → MediaPipe

`pico_hand_state_to_mediapipe()` converts the PICO-native joint state (≥26 × 7)
into the 21-landmark MediaPipe layout in the wrist-centred world frame.
All PICO-specific encoding is removed here; the rest of the pipeline is
input-device agnostic.

### Step 2 — Palm-local normalisation (`_to_palm_local`)

`build_palm_frame_from_landmarks()` constructs an orthonormal frame from the
palm landmarks and `mediapipe_pose @ palm_frame` expresses every landmark
relative to that frame (MANO convention).

This step separates two concerns that are often conflated:

- **Geometric normalisation** — finger positions become relative to the palm,
  independent of wrist position and world orientation.
- **Robot-axis alignment** — done separately in step 3.

Keeping them separate means that adapting to a different robot hand only
requires updating `_R_MANO_TO_ROBOT`; the normalisation logic is unchanged.

### Step 3 — MANO → Robot URDF frame
`mediapipe_local @ _R_MANO_TO_ROBOT` rotates the normalised landmarks into the
coordinate frame expected by the URDF and `ROBOTISHandRetargeter`.

### Step 4 — Retargeting and actuation

`retarget()` solves for robot joint positions that
reproduce the observed finger configuration. The resulting `qpos` vector is
written into `mj_data.ctrl` via the actuator index map built by
`build_qpos_to_actuator_map()`.
---
