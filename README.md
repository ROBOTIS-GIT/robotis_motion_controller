# motion_controller
Motion controller for ROBOTIS Physical AI Lineup

- Install OSQP
```bash
cd ~/
git clone https://github.com/osqp/osqp
cd ~/osqp
mkdir build && cd build
cmake -G "Unix Makefiles" ..
cmake --build .
sudo cmake --build . --target install
```
-  Install osqp-eigen
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
- Set library path
```bash
echo "export OsqpEigen_DIR=/root/osqp-eigen_install" >> ~/.bashrc && \
echo "export LD_LIBRARY_PATH=/root/osqp-eigen_install/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
```
- Install Pinocchio
```bash
sudo apt update
sudo apt install ros-jazzy-pinocchio
```
- Build repository by source
```bash
cd ~/ros2_ws/src
git clone https://github.com/ROBOTIS-GIT/motion_controller.git
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
- Run motion controller(example)
```bash
ros2 launch motion_controller_ros controller.launch.py controller_type:=ai_worker start_interactive_marker:=true
```