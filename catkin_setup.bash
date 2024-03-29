#!/bin/bash
#ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
#libfranka
mkdir ~/robosapiens && cd ~/robosapiens
sudo apt install -y build-essential cmake git libpoco-dev libeigen3-dev
git clone --recursive https://github.com/frankaemika/libfranka && cd libfranka
git checkout 0.8.0 && git submodule update
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build .
cpack -G DEB && sudo dpkg -i libfranka*.deb
#franka_ros
cd ~/robosapiens
mkdir -p catkin_ws/src && cd catkin_ws
sudo apt install ros-noetic-catkin python3-catkin-tools
catkin init
git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros && cd src/franka_ros
git checkout 0.8.0
sed -i "s/_arm.urdf.xacro/\/panda.urdf.xacro arm_id:='panda'/g" ~/robosapiens/catkin_ws/src/franka_ros/franka_control/launch/franka_control.launch
#moveit
cd ~/robosapiens/catkin_ws/src
sudo apt update && sudo apt dist-upgrade -y
sudo apt install -y ros-noetic-catkin python3-catkin-tools python3-wstool python3-rosdep
sudo rosdep init
rosdep update
wstool init .
wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool remove moveit_tutorials
wstool update -t .
#git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel
#configure and build
cd ~/robosapiens/catkin_ws
rosdep install -y --from-paths src --ignore-src --rosdistro noetic --skip-keys libfranka
sudo apt install ros-noetic-desktop-full
catkin config --extend /opt/ros/${ROS_DISTRO} --skiplist franka_description --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/robosapiens/libfranka/build
catkin build
sudo apt install ros-noetic-franka-description ros-noetic-joint-trajectory-controller
echo 'source ~/robosapiens/catkin_ws/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
