from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                 package="gdino_sam",
                 executable="image_processor_node",
                 name="image_processor",
                 output="screen",
             ),
            Node(
                package="gdino_sam",
                executable="prompt_node",
                name="prompt_node",
                output="screen",
            ),
        ]
    )
