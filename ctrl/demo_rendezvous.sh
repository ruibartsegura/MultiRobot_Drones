#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/ros2_ws/install/setup.bash

echo "Press enter to continue: takeoff"; read
ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff "{group_mask: 0, height: 0.5, duration: {sec: 3, nanosec: 0}}"

echo "Press enter to continue: start reynolds_rules_node"; read
LAUNCH=$(ros2 pkg prefix reynold_rules)/share/reynold_rules/launch/reynold_rules.launch.py
PARAM=$(ros2 pkg prefix reynold_rules)/share/reynold_rules/config/params_rendezvous.yaml
LOG=/tmp/reynold_rules_$(date +%Y%m%d-%H%M).log
echo "Launching $LAUNCH ..."
ros2 launch $LAUNCH param_file:=$PARAM & #> $LOG 2>&1 &
echo "Spawned pid $! - logging to $LOG"

echo "Press enter to continue: start rendezvous protocol"; read
ros2 param set /reynold_rules_node nav2point_weight  0.2

echo "Press enter to continue: formation line"; read
ros2 param set /reynold_rules_node nav2point_weight  0.0
ros2 param set /reynold_rules_node formation_type    2
ros2 param set /reynold_rules_node formation_weight  0.3

echo "Press enter to continue: land"; read
./land_all.sh
echo "Press enter to continue: stop reynold_rules_node"; read
killall reynold_rules_node --signal SIGINT

