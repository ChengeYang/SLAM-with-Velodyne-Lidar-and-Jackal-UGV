# Jackal SLAM
Winter Project at Northwestern University


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


## Usage
#### Launch in simulation:
```
roslaunch winter_project simulation.launch
```
#### Launch in Jackal:

* SSH into Jackal, and run:
```
roslaunch winter_project real_jackal.launch
```
* Then change ROS master to Jackal:
```
export ROS_MASTER_URI=http://CPR-J100-0076.local:11311
export ROS_HOSTNAME=robostation.local
```
* Run:
```
roslaunch winter_project real_pc.launch
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

### Direct Ethernet connection
```
nmcli connection up Jackal
ssh administrator@cpr-j100-0076.local
```

### Copy and remove file in Jackal
```
scp -r /home/ethan/jackal_ws/src/winter_project/  administrator@cpr-j100-0076.local:~/chenge_ws/src
sudo rm -r winter_project/
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


## Implementation
### Files on Jackal
* **catkin_ws** workspace for Nate
* **jackal_ws** workspace for Michael
* **chenge_ws** workspace for me
* **/etc/ros/setup.bash** change catkin_make path
* **/etc/ros/indigo/ros.d/** certain launch file to run when pushing the red button

### **twist_mux**
* Assign priorities for different control mode (joystick with highest priority)
* Location: **/winter_project/config/twist_mux_topics.yaml**


## Main issues solved

### Z drifting in rviz
The problem is caused by the default settings of robot_localization package. It can be solved by changing the parameters for EKF in the following files:
* **/jackal_control/config/control.yaml**
* **/jackal_control/config/robot_localization.yaml**
* **/robot_localization/params/ekf_template.yaml**

### Yaw drifting in rviz
The problem is caused by the noise measurements of IMU. The roll and pitch measurements have gravity as the absolute reference (measured by accelerometer), while the yaw measurement does not. Thus, the magnetometer is enabled for package imu_filter_madgwick to provide the absolute yaw reference. The launch file is located at:
* **/etc/ros/indigo/ros.d/base.launch**


### Velodyne Lidar issues
* **VLP-16.urdf.xacro** configuration file for the Velodyne VLP16 Lidar in simulation. Change the param **samples** at the beginning of the file from 1875 to 200. This will significantly improve the FPS of the package in rviz.

* **~/jackal_ws/src/velodyne/velodyne_pointcloud/launch/VLP16_points.launch** configuration file in Jackal for the **velodyne_pointcloud** package that transforms the raw Lidar data to ROS message **sensor_msgs::PointCloud2**. Change the param **rpm** from 600 to 300. This reduces the publish frequency of topic **/velodyne_points**, thus allows the Velodyne VLP16 to sample the surrounding environment for all 360 degrees. This setting solves the issue in rviz that the pointcloud is blinking all the time while each frame contains incomplete surrounding information.
