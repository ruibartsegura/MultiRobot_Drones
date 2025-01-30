ros2 service call /cf_1/notify_setpoints_stop crazyflie_interfaces/srv/NotifySetpointsStop
ros2 service call /cf_2/notify_setpoints_stop crazyflie_interfaces/srv/NotifySetpointsStop
ros2 service call /cf_3/notify_setpoints_stop crazyflie_interfaces/srv/NotifySetpointsStop
ros2 service call /all/land crazyflie_interfaces/srv/Land "{duration: {sec: 3}}"
