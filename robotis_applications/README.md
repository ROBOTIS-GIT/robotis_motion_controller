# robotis_applications

ROS Packages for Robotis Applications

This repository contains the official ROS 2 packages for the ROBOTIS applications. For detailed usage instructions, please refer to the documentation below.
  - [Documentation for Robotis Applications](https://ai.robotis.com/)

For usage instructions and demonstrations of the ROBOTIS Hand, check out:
  - [Tutorial Videos](https://www.youtube.com/@ROBOTISOpenSourceTeam)

To use the Docker image for running ROS packages with the ROBOTIS Hand, visit:
  - [Docker Images](https://hub.docker.com/r/robotis/robotis-applications/tags)

## Launch VR Publisher

The `robotis_vuer` package provides one launch file with a `model` argument.

Run SH5: (default)

```bash
ros2 launch robotis_vuer vr.launch.py model:=sh5
```

Run SG2:

```bash
ros2 launch robotis_vuer vr.launch.py model:=sg2
```
