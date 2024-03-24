# my_launch_file.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('serial_motor_demo'),
        'config',
        'robot_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'speed', default_value='0.3',
            description='Speed for teleop_twist_keyboard'),
        DeclareLaunchArgument(
            'turn', default_value='0.5',
            description='Turn for teleop_twist_keyboard'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0'),

        
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
            parameters=[config],
            
        ),

        Node(
            package='serial_motor_demo',
            executable='wheels_odometry',
            name='wheel_odometry_node',
            output='screen',
            parameters=[config],
        ),
    ])
