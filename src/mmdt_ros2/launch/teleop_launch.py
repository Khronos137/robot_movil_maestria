import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Rutas
    pkg_mmdt_ros2 = get_package_share_directory('mmdt_ros2')
    joy_config = os.path.join(pkg_mmdt_ros2, 'config', 'xbox_teleop.yaml')

    # 1. Nodo del Driver del Joystick (lee el hardware)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0', # Asegúrate que este sea tu joystick
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )

    # 2. Nodo Teleop (convierte joy -> cmd_movil)
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config],
        # AQUÍ HACEMOS LA MAGIA DEL REMAPEO
        # El nodo original publica en /cmd_vel, lo enviamos a /cmd_movil
        remappings=[
            ('/cmd_vel', '/cmd_movil')
        ]
    )

    return LaunchDescription([
        joy_node,
        teleop_node
    ])