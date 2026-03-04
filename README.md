# ROBOTIS Motion Controller

This repository contains motion controller packages for the ROBOTIS Physical AI lineup.

## Repository Structure

```text
├── motion_controller_core/
│   ├── include/motion_controller_core/
│   │   ├── common/
│   │   │   └── ...
│   │   ├── controllers/
│   │   │   └── ...
│   │   ├── kinematics/
│   │   │   └── ...
│   │   ├── optimization/
│   │   │   └── ...
│   │   └── retargeting/
│   │       └── ...
│   └── src/
│       ├── controllers/
│       │   └── ...
│       ├── kinematics/
│       │   └── ...
│       └── retargeting/
│           └── ...
├── CMakeLists.txt
├── package.xml
│
├── motion_controller_ros/
│   ├── config/
│   │   └── ...
│   ├── include/motion_controller_ros/
│   │   ├── nodes/
│   │   │   └── ...
│   │   └── utils/
│   │       └── ...
│   ├── launch/
│   │   └── ...
│   └── src/
│       ├── nodes/
│       │   └── ...
│       └── utils/
│           └── ...
├── CMakeLists.txt
├── package.xml
│
├── motion_controller_models/
│   ├── launch/
│   │   └── ...
│   ├── models/
│   │   └── ...
│   ├── CMakeLists.txt
│   └── package.xml
├── CMakeLists.txt
└── package.xml
```
### Directory Description
| Directory | Description |
|-----------|-------------|
| `motion_controller_core/` | Core package containing kinematics solver & motion control library |
| `motion_controller_core/include/motion_controller_core/common/` | Common headers containing shared types and utility functions |
| `motion_controller_core/include/motion_controller_core/optimization/` | QP definitions and solver interfaces |
| `motion_controller_core/src/controllers/` | Controller implementations |
| `motion_controller_core/src/kinematics/` | Kinematics solver implementation |
| `motion_controller_core/src/retargeting/` | Retargeting module (placeholder, not yet implemented) |

| Directory | Description |
|-----------|-------------|
| `motion_controller_ros/` | ROS 2 package containing controller nodes, launch files, and configs |
| `motion_controller_ros/launch/` | Launch files for running the controller nodes |
| `motion_controller_ros/src/nodes/` | ROS 2 node executables for each controller type |
| `motion_controller_ros/src/utils/` | Utility nodes |

| Directory | Description |
|-----------|-------------|
| `motion_controller_models/` | Robot model descriptions package |
| `motion_controller_models/launch/` | Launch files for visualizing robot models |
| `motion_controller_models/models/` | URDF/SRDF robot models used by the controller |

## Install (from source)

### Prerequisites

- ROS 2 Jazzy installed

### Install OSQP

```bash
cd ~/
git clone https://github.com/osqp/osqp
cd ~/osqp
mkdir build && cd build
cmake -G "Unix Makefiles" ..
cmake --build .
sudo cmake --build . --target install
```

### Install OsqpEigen (osqp-eigen)

```bash
cd ~/
mkdir osqp-eigen_install
git clone https://github.com/robotology/osqp-eigen.git
cd ~/osqp-eigen
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX:PATH=~/osqp-eigen_install ../
make
sudo make install
```

### Set library path (for OsqpEigen)

```bash
echo "export OsqpEigen_DIR=$HOME/osqp-eigen_install" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$HOME/osqp-eigen_install/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc
```

### Install Pinocchio

```bash
sudo apt update
sudo apt install -y ros-jazzy-pinocchio
```

### Build in a ROS 2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/ROBOTIS-GIT/motion_controller.git
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# AI Worker controller is launched as default
ros2 launch motion_controller_ros controller.launch.py controller_type:=ai_worker start_interactive_marker:=true
```

You can also switch controllers via `controller_type`:

- `controller_type:=joint_space` (runs `joint_space_controller_node`)
- `controller_type:=leader` (runs `leader_controller_node` and also starts the follower controller for leader/follower use)