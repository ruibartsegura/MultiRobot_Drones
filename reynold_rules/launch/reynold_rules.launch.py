import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Obtener el directorio del paquete
    pkg_dir = get_package_share_directory('reynold_rules')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')  # Por si necesitas este archivo más adelante

    # Nodo de Reynold
    reynold_cmd = Node(
        package='reynold_rules',
        executable='reynold_rules_node',
        output='screen',
        # prefix=['/usr/bin/valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --verbose'],
        remappings=[]
    )
    
    takeoff_service_call = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/all/takeoff',
            'crazyflie_interfaces/srv/Takeoff',
            '{"group_mask": 0, "height": 0.5, "duration": {"sec": 5, "nanosec": 0}}'
        ],
        output='screen'
    )


    # Crear LaunchDescription y agregar las acciones
    ld = LaunchDescription()
    ld.add_action(reynold_cmd)
    ld.add_action(takeoff_service_call)

    return ld
