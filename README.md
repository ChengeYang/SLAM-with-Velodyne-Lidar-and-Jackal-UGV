# Jackal SLAM
Winter Project, Northwestern University

## Introduction


## Dependencies
The Jackal UGV packages are released in ROS Indigo and Kinetic. To use them in ROS Melodic, the following compiling processes are implemented:

#### Build from apt-get install:
Install in Terminal with **sudo apt-get install**:
* ros-melodic-velodyne
* ros-melodic-velodyne-description
* ros-melodic-velodyne-simulator
* ros-melodic-geographic-info
* ros-melodic-robot-localization
* ros-melodic-twist-mux

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
* Launch Gazebo with front_laser config:
```
roslaunch jackal_gazebo jackal_world.launch config:=front_laser
```
* Launch rviz to control the Jackal in Gazebo:
```
roslaunch jackal_viz view_robot.launch
```
* Drive the Jackal: send message type **geometry_msgs/Twist** to topic **cmd_vel**.


## Initial Setup
### Router connection
* Connect to **JACKALROUTER24**; Password: **jackbenimble**
```
nmcli connection up JACKALROUTER24
```
* SSH into Jackal
```
ssh administrator@192.168.0.100
ssh administrator@cpr-j100-0076.local
Password: clearpath
cd jackal_ws/src/winter_project/
source remote-jackal.sh
```

### Direct wireless connection
```
nmcli connection up JackalAdHoc
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

### Files on Jackal
* **catkin_ws** workspace for Nate
* **jackal_ws** workspace for Michael
* **/etc/ros/indigo/ros.d/** certain launch file to run when hitting the red button
* **nu_jackal_autonav_startup.launch** write launch packages in this file if you want it to be launched every time you stitch on the robot. (Now Velodyne is launched)
* **/etc/ros/setup.bash** change ROS workspace path

### **twist_mux**
* Assign priorities for different control mode (joystick with highest priority)
* Location: **params/twist_mux_topics.yaml**
