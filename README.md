# MuscleMemory_for_HighPrecisionLocomotion

## Overview

This file documents the code of control design of the Hybrid Control Framework with muscle memory.

## Introduction:
This work proposed a scalable and adaptive control framework for legged robots that integrates Iterative Learning Control (ILC) with a biologically inspired torque library (TL), analogous to muscle memory. The proposed method addresses key challenges in robotic locomotion, including the accurate tracking of trajectories under unmodeled dynamics and external disturbances.
We developed a Hybrid Control Framework with a generalized TL that stores learned control profiles and enables rapid adaptation to changes in speed, terrain, and gravitational conditions—eliminating the need for repeated learning and significantly reducing online computation.
The approach is validated on the bipedal robot Cassie and the quadrupedal robot A1 through extensive simulations and hardware experiments.  
## Video
[![Watch the video](https://raw.githubusercontent.com/DLARlab/MuscleMemory_for_HighPrecisionLocomotion/main/Media/mediademo_thumbnail.png)](https://drive.google.com/file/d/1z0UsrwpdnCYdf63Tudqv5o2jNVuLqCG7/view?usp=sharing)

This is research code, expect that it changes often, and any fitness for a particular purpose is disclaimed.
The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Jing Cheng, Yasser G. Alqaham, Zhenyu Gan and Amit K. Sanyal<br />
Affiliation: [DLAR Lab](https://dlarlab.syr.edu)<br />
Maintainer: Jing Cheng, jcheng13@syr.edu />**
With contributions by: Unitree(https://github.com/unitreerobotics/unitree_ros)

This project was initially developed at Syracuse University (Dynamic Locomotion and Robotics Lab).

## Publications

This work has been submitted to the Robotica.

If you use this work in an academic context, please cite the following publication:

## Implementation Requirements
This package can send control commands to A1 robot from ROS based on the control.
This version is suitable for unitree_legged_sdk v3.2 and v3.1.

Basic message function: `unitree_legged_msgs`
The interface between ROS and real robot: `unitree_legged_real`

# Dependencies
* [unitree_legged_sdk](https://github.com/unitreerobotics):

# Configuration
Make sure the following exist in your `~/.bashrc` file or export them in terminal. `melodic`, `gazebo-8`, `~/catkin_ws`, `amd64` and the paths to `unitree_legged_sdk` should be replaced in your own case. 
If your use `unitree_legged_sdk`, then you need to set `UNITREE_SDK_VERSION=3_2` and the path `UNITREE_LEGGED_SDK_PATH`.
Otherwise, if you use `aliengo_sdk`, you need to set `UNITREE_SDK_VERSION=3_1` and the path `ALIENGO_SDK_PATH`.

```
source /opt/ros/melodic/setup.bash
source /usr/share/gazebo-8/setup.sh
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=~/catkin_ws:${ROS_PACKAGE_PATH}
export GAZEBO_PLUGIN_PATH=~/catkin_ws/devel/lib:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=~/catkin_ws/devel/lib:${LD_LIBRARY_PATH}
# 3_1, 3_2
export UNITREE_SDK_VERSION=3_2
export UNITREE_LEGGED_SDK_PATH=~/unitree_legged_sdk
export ALIENGO_SDK_PATH=~/aliengo_sdk
# amd64, arm32, arm64
export UNITREE_PLATFORM="amd64"
```

# Build
Please run the following command to install relative packages.

If your ROS is melodic:
```
sudo apt-get install ros-melodic-controller-interface  ros-melodic-gazebo-ros-control ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller
```
Else if your ROS is Noetic:
```
sudo apt-get install ros-noetic-controller-manager ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-velocity-controllers ros-noetic-position-controllers ros-noetic-robot-controllers ros-noetic-robot-state-publisher
```

And open the file `unitree_gazebo/worlds/stairs.world`. At the end of the file:
```
<include>
    <uri>model:///home/unitree/catkin_ws/src/unitree_ros/unitree_gazebo/worlds/building_editor_models/stairs</uri>
</include>
```
Please change the path of `building_editor_models/stairs` to the real path on your PC.
You can use catkin_make to build ROS packages. First copy the package folder to `~/catkin_ws/src`, then:
```
cd ~/catkin_ws
catkin_make
```
Before compiling `unitree_legged_real`, please make sure that the `unitree_legged_msgs` has been compiled.

# Setup the net connection
First, please connect the network cable between your PC and robot. Then run `ifconfig` in a terminal, you will find your port name. For example, `enx000ec6612921`.

Then, open the `ipconfig.sh` file under the folder `unitree_legged_real`, modify the port name to your own. And run the following commands:
```
sudo chmod +x ipconfig.sh
sudo ./ipconfig.sh
```
If you run the `ifconfig` again, you will find that port has `inet` and `netmask` now.
In order to set your port automatically, you can modify `interfaces`:
```
sudo gedit /etc/network/interfaces
```
And add the following 4 lines at the end:
```
auto enx000ec6612921
iface enx000ec6612921 inet static
address 192.168.123.162
netmask 255.255.255.0
```
Where the port name has to be changed to your own.

# Run the package
You can control your real robot from ROS by this package.

First you have to run the `real_launch` under root account:
```
sudo su
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
source /home/yourUserName/catkin_ws/devel/setup.bash
roslaunch unitree_legged_real real.launch rname:=a1 ctrl_level:=highlevel firmwork:=3_2
```
Please note that `/home/yourUserName` refers to your own home directory.

These commands will launch an LCM server.  
- `rname` stands for the robot name, which can be `a1` or `aliengo` (case-insensitive). The default value is `a1`.  
- `ctrl_level` specifies the control level and can be either `lowlevel` or `highlevel`.  
- `firmwork` refers to the firmware version of the robot. The default value is `3_2`. Currently, all A1 robots use firmware version `3_2`.

Please also double-check the file paths on your computer, as incorrect directories may not trigger an error message in the terminal.

In order to send message to robot, you need to run the controller in another terminal(also under root account):
```
rosrun unitree_legged_real TL_ILC_lcm
```
While the controller is running, use **rosbag** to record the data in another terminal.

Use `ILC_Switch[i]` to control the robot in ILC mode (`ILC_Switch[0] = 1`) or in TL mode (`ILC_Switch[1] = 1`).

After the robot successfully starts, it will stand up and run into PD control mode. The ILC or TL controller will be enabled after the PD time has finished.

You can use the parameter `PD_time` to set the duration of the PD controller.

