import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('reynold_rules')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    reynold_cmd = Node(package='reynold_rules',
                          executable='reynold_rules_class',
                          output='screen',
                          prefix=['/usr/bin/valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --verbose'],
                          remappings=[])


    ld = LaunchDescription()
    ld.add_action(reynold_cmd)

    return ld
