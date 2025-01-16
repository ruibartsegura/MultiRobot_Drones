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

    # Ruta al archivo .sh
    startup_script = os.path.expanduser('~/CrazySim/ros2_ws/src/mrs_crazyflies/startup/start.sh')

    # Proceso para ejecutar el script .sh
    startup_process = ExecuteProcess(
        cmd=['/bin/bash', startup_script],
        output='screen'
    )

    # Crear LaunchDescription y agregar las acciones
    ld = LaunchDescription()
    ld.add_action(reynold_cmd)      # Agregar el nodo Reynold
    ld.add_action(startup_process)  # Agregar el script .sh

    return ld
