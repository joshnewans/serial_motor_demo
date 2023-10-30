# my_launch_file.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'speed', default_value='0.5',
            description='Speed for teleop_twist_keyboard'),
        DeclareLaunchArgument(
            'turn', default_value='1.0',
            description='Turn for teleop_twist_keyboard'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0'),

        
        Node(
            package='serial_motor_demo',
            executable='driver',
            name='driver',
            output='screen',
            parameters=[
                {'serial_port': LaunchConfiguration('serial_port')},
            ],
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            parameters=[
                {'speed': LaunchConfiguration('speed')},
                {'turn': LaunchConfiguration('turn')},
            ],
        ),
        Node(
            package='serial_motor_demo',
            executable='motor_command_node',
            name='motor_command_node',
            output='screen',
            
        ),
    ])
