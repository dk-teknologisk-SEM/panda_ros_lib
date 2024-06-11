
# TOC
1. [Installation](#install)
1. [How to run](#run)
1. [How to add robot controllers](#add-controller)
1. [How to add/use custom TCPs](#adduse-tcps)
1. [Example](#example)
1. [Help](#help)

## Install
1. Install Ubuntu 20.04 Focal Fossa: [Guide](https://ubuntu.com/tutorials/install-ubuntu-desktop)
1. Install ROS Noetic: [Guide](https://wiki.ros.org/noetic/Installation/Ubuntu)
    1.```sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'```
    1. ```sudo apt install -y curl```
    1. ```curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -```
    1. ```sudo apt update```
    1. ```sudo apt install ros-noetic-desktop-full```
    1. ```echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc```
1. Install LibFranka: [Guide](https://frankaemika.github.io/docs/installation_linux.html#building-libfranka)
    1. ```mkdir ~/robosapiens && cd ~/robosapiens```
    1. ```sudo apt install build-essential cmake git libpoco-dev libeigen3-dev```
    1. ```git clone --recursive https://github.com/frankaemika/libfranka && cd libfranka```
    1. ```git checkout 0.8.0 && git submodule update```
    1. ```mkdir build && cd build```
    1. ```cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..```
    1. ```cmake --build .```
    1. ```cpack -G DEB && sudo dpkg -i libfranka*.deb```

1. Install franka-ros[Guide](https://frankaemika.github.io/docs/installation_linux.html#building-the-ros-packages)
    1. ```cd ~/robosapiens```
    1. ```mkdir -p catkin_ws/src && cd catkin_ws```
    1. ```source /opt/ros/noetic/setup.sh```
    1. ```catkin init src```
    1. ```git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros && cd src/franka_ros```
    1. ```git checkout 0.8.0```
    1. ```sed -i "s/_arm.urdf.xacro/\/panda.urdf.xacro arm_id:='panda'/g" ~/robosapiens/catkin_ws/src/franka_ros/franka_control/launch/franka_control.launch```
    <!-- 1. ```catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/robosapiens/libfranka/build``` -->

1. Install MoveIt: [Guide](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
    1. ```cd ~/robosapiens/catkin_ws/src```
    1. ```sudo apt update && sudo apt dist-upgrade```
    1. ```sudo apt install -y ros-noetic-catkin python3-catkin-tools python3-wstool python3-rosdep```
    1. ```sudo rosdep init```
    1. ```rosdep update```
    1. ```wstool init .```
    1. ```wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall```
    1. ```wstool remove moveit_tutorials```
    1. ```wstool update -t .```
    <!-- 1. ```git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel``` -->
    1. ```cd ~/robosapiens/catkin_ws```
    1. ```rosdep install -y --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka```
    1. ```catkin config --extend /opt/ros/${ROS_DISTRO} --skiplist franka_description --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/robosapiens/libfranka/build```
    1. ```catkin build```
    1. ```sudo apt install ros-noetic-desktop-full ros-noetic-franka-description ros-noetic-joint-trajectory-controller```
    1. ```source /opt/ros/noetic/setup.bash```
    1. ```source ~/robosapiens/catkin_ws/devel/setup.bash```
    1. ```echo 'source ~/robosapiens/catkin_ws/devel/setup.bash' >> ~/.bashrc```

1. Install Real Time Kernel: [Guide](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)
    1.```mkdir -p ~/robosapiens/rt-kernel && cd ~/robosapiens/rt-kernel``` 
    1. ```sudo apt-get install -y build-essential bc curl ca-certificates gnupg2 libssl-dev lsb-release libelf-dev bison flex dwarves zstd libncurses-dev```
    1. ```curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.xz```
    1. ```curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.sign```
    1. ```curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.xz```
    1. ```curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.sign```
    1. ```xz -d *.xz```
    1. ```gpg2  --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 647F28654894E3BD457199BE38DBBDC86092693E```
    1. ```gpg2 --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 57892E705233051337F6FDD105641F175712FA5B```
    1. ```gpg2 --verify linux-*.tar.sign```
    1. ```gpg2 --verify patch-*.patch.sign```
    1. ```tar xf linux-*.tar```
    1. ```cd linux-*/```
    1. ```patch -p1 < ../patch-*.patch```
    1. ```cp -v /boot/config-$(uname -r) .config```
    1. ```make olddefconfig```
    1. ```make xconfig```
    1. General Setup > Preemption Model and select Fully Preemptible Kernel (Real-Time)
    1. After that navigate to Cryptographic API > Certificates for signature checking (at the very bottom of the list) > Provide system-wide ring of trusted keys > Additional X.509 keys for default system keyring
    1. Remove the “debian/canonical-certs.pem” from the prompt and press Ok. Save this configuration to .config and exit the GUI.
    1. ```make -j$(nproc) deb-pkg```
    1. ```sudo dpkg -i ../linux-headers-*.deb ../linux-image-*.deb```
    1. ```reboot and choose realtime kernel```
    1. ```sudo addgroup realtime```
    1. ```sudo usermod -a -G realtime $(whoami)```
    1. add the following limits to the realtime group in /etc/security/limits.conf
    ```
    @realtime soft rtprio 99
    @realtime soft priority 99
    @realtime soft memlock 102400
    @realtime hard rtprio 99
    @realtime hard priority 99
    @realtime hard memlock 102400
    ```

1. Install panda_ros_lib
    1. ```cd ~/robosapiens```
    1. Clone Code: ```git clone git@github.com:dk-teknologisk-SEM/panda_ros_lib.git```

## Run
1. Robot Connection
    1. New terminal
    1. source stuff
    1. roslaunch panda_moveit_config franka_control.launch robot_ip:={ROBOT_IP}
1. Robot Controller
    1. New terminal
    1. source stuff
    1. ```python3 demo.py```



## Add controller
Controllers can be added to your program by first adding the new controller to default_controllers.yaml () and then loading it in ros_controllers.launch ().

NOTE: only 1! controller can be active at any one time, therefore all but 1 of the loaded controllers in ros_controllers.launch needs the "--stopped" argument.

![default_controllers](/img/default_controllers.png)
![ros_controllers](/img/ros_controllers.png)

Once the controller has been loaded, you can start/stop them by calling ```arm.start_controller``` and ```arm.stop_controller```.

A list of all possible controllers can be obtained by either calling ```arm.get_controllers``` or by manually calling the ```/controller_manager/list_controllers``` ros service

## Add/Use TCPs
To add custom TCPs to your program, you need to add the TCP info to the robot URDF file, it can technically be in any of the included files, but I suggest that you add it to franka_hand.xacro file () 

Each TCP needs to reference the default franka hand TCP (or robot flange) as it parent to place it on the robot correctly.

![franka_hand](/img/franka_hand.png)

NOTE: All programs need to be restarted for theese changes to take effect.

After the program has been started, the active TCP can be set by calling ```arm.move_group.set_end_effector_link(end_effector_link)``` with ```end_effector_link``` being the name of the link in the URDF file, in combination with ```arm.set_EE_frame``` and potentially ```arm.set_load``` depending on the tool;

these functions are documented in the code.

## Example
See [here](https://github.com/dk-teknologisk-SEM/RB24-robosapiens)

## Help
Contact [MDIH](https://github.com/dk-teknologisk-mdih) or [SEM](https://github.com/dk-teknologisk-sem)