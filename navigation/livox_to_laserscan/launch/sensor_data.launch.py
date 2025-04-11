from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='livox_to_laserscan',
            executable='livox_to_laserscan',
            name='lidar_to_laserscan',
            output='screen'
        ),
        Node(
            package='livox_to_laserscan',
            executable='custom_tf_pub',
            name='tf_publisher',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('livox_ros_driver2'),
                '/launch/msg_MID360_launch.py'
            ])
        )
    ])
