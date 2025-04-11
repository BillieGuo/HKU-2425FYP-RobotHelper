from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    livox_to_laserscan = Node(
        package='livox_to_laserscan',
        executable='livox_to_laserscan',
        name='lidar_to_laserscan',
        output='screen'
    )
    
    custom_tf = Node(
        package='livox_to_laserscan',
        executable='custom_tf_pub',
        name='tf_publisher',
        output='screen'
    )
    
    livox_driver_dir = get_package_share_directory('livox_ros_driver2')
    mid_360 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                livox_driver_dir,
                'launch_ROS2',
                'msg_MID360_launch.py'
            )
        )
    )
    
    return LaunchDescription([
        livox_to_laserscan,
        custom_tf,
        mid_360
    ])
