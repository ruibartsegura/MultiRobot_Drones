#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/ros2_ws/install/setup.bash

echo "Press enter to continue: takeoff"; read
ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff "{group_mask: 0, height: 0.5, duration: {sec: 3, nanosec: 0}}"

echo "Press enter to continue: start reynolds_rules_node"; read
LAUNCH=$(ros2 pkg prefix reynold_rules)/share/reynold_rules/launch/reynold_rules.launch.py
LOG=/tmp/reynold_rules_$(date +%Y%m%d-%H%M).log
echo "Launching $LAUNCH ..."
ros2 launch $LAUNCH & #> $LOG 2>&1 &
proc_id=$!
echo "Spawned pid $! - logging to $LOG"

end="n";
while [ $end != "y" ]; do
    echo "Enter x and y coord. Press enter to continue"; read x y
    ros2 param set /reynold_rules_node nav2point_weight  0.2
    ros2 param set /reynold_rules_node target_point      [$x,$y,1.0]

    echo "Press enter to continue: formation line"; read type
    ros2 param set /reynold_rules_node nav2point_weight  0.0
    ros2 param set /reynold_rules_node formation_type    $type

    echo "Do you want to finish? (y/n): "; read end
done

echo "Press enter to continue: stop reynold_rules_node"; read
killall reynold_rules_node --signal SIGINT

echo "Press enter to continue: land"; read
kill $proc_id
./land_all.sh
