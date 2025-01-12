import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    kobuki_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('kobuki'),
            'launch',
            'simulation.launch.py')),
        )

    pkg_dir = get_package_share_directory('reynold_rules')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    followball_cmd = Node(package='reynold_rules',
                          executable='reynold_rules_node',
                          output='screen',
                          parameters=[param_file],
                          remappings=[
                            ('target', '/pos_target_pub'),
                            ('obstacle', '/pos_obstacle_pub'),
                          ])


    ld = LaunchDescription()
    ld.add_action(kobuki_cmd)
    ld.add_action(camera_cmd)

    return ld
