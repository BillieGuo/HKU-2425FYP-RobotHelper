from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='translator',
            executable='socket_server',
            name='socket_server',
            output='screen'
        ),
        Node(
            package='translator',
            executable='master',
            name='master',
            output='screen'
        )
    ])
