#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/ros2_ws/install/setup.bash

print "NotifySetpointsStop"
ros2 service call /cf_1/notify_setpoints_stop crazyflie_interfaces/srv/NotifySetpointsStop &
pid_1=$!
ros2 service call /cf_2/notify_setpoints_stop crazyflie_interfaces/srv/NotifySetpointsStop &
pid_2=$!
ros2 service call /cf_3/notify_setpoints_stop crazyflie_interfaces/srv/NotifySetpointsStop &
pid_3=$!

wait $pid_1
wait $pid_2
wait $pid_3

print "Land"
ros2 service call /cf_1/land crazyflie_interfaces/srv/Land "{duration: {sec: 1}}" &
ros2 service call /cf_2/land crazyflie_interfaces/srv/Land "{duration: {sec: 1}}" &
ros2 service call /cf_3/land crazyflie_interfaces/srv/Land "{duration: {sec: 1}}"

