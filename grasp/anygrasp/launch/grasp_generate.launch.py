from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="anygrasp",
                executable="anygrasp_node",
                name="anygrasp_node",
                output="screen",
             ),
            Node(
                package="anygrasp",
                executable="grasp_responder",
                name="grasp_responder",
                output="screen",
            ),
        ]
    )
