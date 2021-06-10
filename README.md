# Motion-planning-for-VIPs
Three different motion planning algorithms have been modified and implemented to meet blind people's requirements:
- Dijkstra
- RRT*
- Genetic Algorithm

These have been integrated with our version of ROS Navigation stack and the system has been evaluated with turtlebot.
Finally it has been integrated with the CUFF an haptic and wearable device with the goal of send stimuli/commands to visually impaired helping them in their navigation. ![](RRTstar and cuff.gif)

### Required Packages

- ROS packages:
  - sudo apt install ros-[ROS-VERSION]-navigation
  - sudo apt install ros-[ROS-VERSION]-realsense2-camera
  - sudo apt install ros-[ROS-VERSION]-realsense2-description
  - sudo apt install ros-[ROS-VERSION]-imu-tools
  - sudo apt install ros-[ROS-VERSION]-imu-filter-madgwick
  - sudo apt install ros-[ROS-VERSION]-robot-pose-ekf
  - sudo apt install ros-[ROS-VERSION]-depthimage-to-laserscan

- LibRealSense2:
  - sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
  - sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo UBUNTU_VERSION main" -u
  - sudo apt install librealsense2-dkms
  - sudo apt install librealsense2-utils
  - sudo apt install librealsense2-dev
  - sudo apt install librealsense2-dbg

**ACHTUNG** ros-[ROS-VERSION]-depthimage-to-laserscan is not provided for *Noetic* distro (update 31-March-2021) 

### RealSense2

- [LibRealSense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)
- [RealSense2 on GitHub](https://github.com/IntelRealSense/realsense-ros)
