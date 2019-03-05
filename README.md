# Jackal SLAM
Winter Project, Northwestern University

## Introduction


## Dependencies
The Jackal UGV packages are released in ROS Indigo and Kinetic. To use them in ROS Melodic, the following compiling processes are implemented:

#### Build from apt-get:
Install in Terminal with **sudo apt-get install**:
* ros-melodic-velodyne
* ros-melodic-velodyne-description
* ros-melodic-velodyne-simulator
* ros-melodic-geographic-info
* ros-melodic-robot-localization
* ros-melodic-twist-mux
* ros-melodic-pointcloud-to-laserscan

#### Build from source: (version: kinetic)
Git clone the original Github repo to local catkin workspace, and run **catkin_make**.
* [jackal_desktop](http://wiki.ros.org/jackal_desktop)
* [jackal_simulator](http://wiki.ros.org/jackal_simulator)
* [jackal_navigation](http://wiki.ros.org/jackal_navigation)
* [lms1xx](http://wiki.ros.org/LMS1xx)
* [pointgrey_camera_driver](http://wiki.ros.org/pointgrey_camera_driver)
* [interactive_marker_twist_server](http://wiki.ros.org/interactive_marker_twist_server)
* [gmapping](http://wiki.ros.org/gmapping)
* [openslam_gmapping](http://wiki.ros.org/openslam_gmapping)


## Basic Usage
* Launch in simulation:
```
roslaunch winter_project simulation.launch
```


## Initial Setup
### Router connection
* Connect to **JACKALROUTER24**; Password: **jackbenimble**
```
nmcli connection up JACKALROUTER24
```
* SSH into Jackal
```
ssh administrator@192.168.0.100
Password: clearpath
```

### Direct wireless connection
```
nmcli connection up Jackal
ssh administrator@cpr-j100-0076.local
```

### PS3 Joystick
* Plug into the laptop will erase the memory. Reset up required.
* Connect the joystick to the Jackal through USB
* SSH into jackal
* Run
```
sudo sixpair
sudo sixad --boot-yes
```
* Unplug the joystick, long hold the playstation button

### Copy and remove file in Jackal
```
scp -r /home/ethan/jackal_ws/src/winter_project/  administrator@cpr-j100-0076.local:~/chenge_ws/src
sudo rm -r winter_project/
```


## Implementation
### Files on Jackal
* **catkin_ws** workspace for Nate
* **jackal_ws** workspace for Michael
* **/etc/ros/indigo/ros.d/** certain launch file to run when hitting the red button
* **nu_jackal_autonav_startup.launch** write launch packages in this file if you want it to be launched every time you switch on the robot. (Now Velodyne is launched while the machine vision camera is not)
* **/etc/ros/setup.bash** change ROS workspace path

### **twist_mux**
* Assign priorities for different control mode (joystick with highest priority)
* Location: **/config/twist_mux_topics.yaml**

### IMU issues
The Jackal is drifting in Gazebo when running in real world. The problem can be solved by changing the parameters for Kalman Filter in the following files.
* **/jackal_control/config/control.yaml**
* **/jackal_control/config/robot_localization.yaml**
* **/robot_localization/params/ekf_template.yaml**

### Velodyne Lidar issues
* **VLP-16.urdf.xacro** configuration file for the Velodyne VLP16 Lidar in simulation. Change the param **samples** at the beginning of the file from 1875 to 200. This will significantly improve the efficiency of the package in rviz.
* **~/jackal_ws/src/velodyne/velodyne_pointcloud/launch/VLP16_points.launch** configuration file in Jackal for the **velodyne_pointcloud** package that transforms the raw Lidar data to ROS message **sensor_msgs::PointCloud2**. Change the param **rpm** from 600 to 300. This reduces the publish frequency of topic **/velodyne_points**, thus allows the Velodyne VLP16 to sample the surrounding environment for all 360 degrees. This setting solves the issue in rviz that the pointcloud is blinking all the time while each frame contains incomplete surrounding information.
