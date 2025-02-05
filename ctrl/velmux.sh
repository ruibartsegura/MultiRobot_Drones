#!/bin/bash

source ~/.bashrc
source /opt/ros/iron/setup.bash
source /home/developer/ros2_ws/install/setup.bash
cd ros2_ws/
ros2 launch mrs_crazyflies_exp cf_velmux_launch.py
