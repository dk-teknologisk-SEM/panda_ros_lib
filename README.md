# Install
1. Install Ubuntu 20.04 Focal Fossa: [Guide](https://ubuntu.com/tutorials/install-ubuntu-desktop)
1. Install ROS Noetic: [Guide](https://wiki.ros.org/noetic/Installation/Ubuntu)
1. Install LibFranka: [Guide](https://frankaemika.github.io/docs/installation_linux.html#building-libfranka)
    1. ```mkdir ~/robosapiens && cd ~/robosapiens```
    1. ```sudo apt install build-essential cmake git libpoco-dev libeigen3-dev```
    1. ```git clone --recursive https://github.com/frankaemika/libfranka && cd libfranka```
    1. git checkout 0.8.0 && git submodule update
    1. mkdir build && cd build
    1. cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
    1. cmake --build .
    1. cpack -G DEB && sudo dpkg -i libfranka*.deb
1. Install franka-ros[Guide](https://frankaemika.github.io/docs/installation_linux.html#building-the-ros-packages)
    1. cd ~/robosapiens
    1. mkdir -p catkin_ws/src && cd catkin_ws
    1. source /opt/ros/noetic/setup.sh
    1. catkin_init_workspace src
    1. git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
    1. git checkout 0.8.0
    1. rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
    1. catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/robosapiens/libfranka/build
1. Install Real Time Kernel: [Guide](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)
    1. sudo apt-get install build-essential bc curl ca-certificates gnupg2 libssl-dev lsb-release libelf-dev bison flex dwarves zstd libncurses-dev
    1. curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.xz
    1. curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.sign
    1. curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.xz
    1. curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.sign
    1. xz -d *.xz
    1. gpg2  --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 6092693E
    1. gpg2 --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 2872E4CC
    1. gpg2 --verify linux-*.tar.sign
    1. gpg2 --verify patch-*.patch.sign
    1. tar xf linux-*.tar
    1. cd linux-*/
    1. patch -p1 < ../patch-*.patch
    1. cp -v /boot/config-$(uname -r) .config
    1. make olddefconfig
    1. make xconfig
    1. General Setup > Preemption Model and select Fully Preemptible Kernel (Real-Time)
    1. After that navigate to Cryptographic API > Certificates for signature checking (at the very bottom of the list) > Provide system-wide ring of trusted keys > Additional X.509 keys for default system keyring

    1. Remove the “debian/canonical-certs.pem” from the prompt and press Ok. Save this configuration to .config and exit the GUI.
    1. make -j$(nproc) deb-pkg
    1. sudo dpkg -i ../linux-headers-*.deb ../linux-image-*.deb
    1. reboot and choose realtime kernel
    1. sudo addgroup realtime
    1. sudo usermod -a -G realtime $(whoami)
    1. add the following limits to the realtime group in /etc/security/limits.conf
    ```
    @realtime soft rtprio 99
    @realtime soft priority 99
    @realtime soft memlock 102400
    @realtime hard rtprio 99
    @realtime hard priority 99
    @realtime hard memlock 102400
    ```

1. Install MoveIt: [Guide](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
1. Clone Code: ```git clone git@github.com:dk-teknologisk-SEM/panda_ros_lib.git```

# Run
1. Robot Connection
    1. New terminal
    1. source stuff
    1. roslaunch panda_moveit_config franka_control.launch robot_ip:={ROBOT_IP}
1. Robot Controller
    1. New terminal
    1. source stuff
    1. ```python3 demo.py```