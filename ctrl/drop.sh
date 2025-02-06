ros2 service call /cf_1/land crazyflie_interfaces/srv/Land "{duration: {sec: 1}}" &
ros2 service call /cf_2/land crazyflie_interfaces/srv/Land "{duration: {sec: 1}}" &
ros2 service call /cf_3/land crazyflie_interfaces/srv/Land "{duration: {sec: 1}}"
