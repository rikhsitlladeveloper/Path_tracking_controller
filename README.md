# Path_tracking_controller
This repository provides a path tracking controller for simulated vehicle model of Polaris GEM e2 Electric Cart in the Gazebo simulation environment .Vehicle in the simulation follows path created by path node and publishes cross_track_error topic for checking cleareance of path and moving vehicle. 

## Requirements
Polaris GEM e2 Simulator must be installed. Here is link to install it 
https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2#polaris-gem-e2-simulator

System: Ubuntu 20.04 + ROS Noetic (Gazebo 11)

We refer readers to http://wiki.ros.org/noetic/Installation/Ubuntu and follow the instructions to install ROS noetic and Gazebo 11.
We also recommend Desktop-Full Install as suggested in the instructions.

## Required ROS Packages:

- ackermann_msgs
- geometry2 
- hector_gazebo
- hector_models
- jsk_rviz_plugins
- ros_control
- ros_controllers
- velodyne_simulator

Here is Installation of  Path Tracking Controller with Polaris GEM e2 Simulator 

```
    $ sudo apt install ros-noetic-ackermann-msgs ros-noetic-geometry2 \
    ros-noetic-hector-gazebo ros-noetic-hector-models ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-velodyne-simulator

```
```
    $ mkdir -p ~/gem_ws/src
    $ cd ~/gem_ws/src
    $ git clone https://github.com/rikhsitlladeveloper/Path_tracking_controller.git
    $ git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git
```
```
    $ source /opt/ros/noetic/setup.bash
    $ cd ~/gem_ws
    $ catkin_make


```
# Usage

```
$ source devel/setup.bash
$ roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="true"
$ roslaunch path_tracking_controller path_tracking.launch 
```

# You can check rqt plot of cross track error with following command
```
    $ rosrun rqt_plot rqt_plot 
```
