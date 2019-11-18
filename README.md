[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
# robot_walker

## Overview

This repository simulates turtlebot in gazebo which has a defined map of environment. The robot navigates in the map by avoiding obstacles in its path. The robot keeps going straight until its in proximity of obstacle then rotates when it approaches obstacle. The commands are sent to "/mobile_base/commands/velocity" this topic. This project uses ROS,Catkin,Gazebo libraries as key libraries.Here, laserscan is used to get the sensor data.

## Dependencies
ROS Distro : Kinetic\
Ubuntu 16.04\
Turtlebot packages To install turtlebot, type the following:

sudo apt-get install ros-kinetic-turtlebot-gazebo\
ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers

## Build instructions in terminal
source /opt/ros/kinetic/setup.bash\
mkdir -p ~/catkin_ws/src\
cd catkin_ws\
source devel/setup.bash\
cd src\
git clone https://github.com/iskalasrinivas/robot_walker \
cd ..\
catkin_make

## Cpplint check
Execute the following commands in a new terminal to run cpplint

cd  <path to repository>
cpplint $( find . -name \*.hpp -or -name \*.cpp )

## Running demo

## Using roslaunch
Type the following command in a new terminal:

roslaunch robot_walker walkerDemo.launch

## Recording using rosbag files
Record the rostopics using the following command with the launch file:

roslaunch robot_walker walkerDemo.launch record:=true\
recorded bag file will be stored in the results folder present in the package

To record for a specific time

roslaunch robot_walker walkerDemo.launch record:=true secs:=20\
In the above case rosbag will record for 20 seconds

## Playing bag files
navigate to the results folder

cd ~/catkin_ws/src/turtlebot_walker/results \ 
play the bag file

rosbag play record.bag

Can verify the published topic by echoing the following topic

rostopic echo /mobile_base/commands/velocity 

