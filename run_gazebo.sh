#!/bin/bash

sudo killall rosmaster
sudo killall gzserver
sudo killall gzclient
source devel/setup.bash
roslaunch robot_gazebo simulation.launch
