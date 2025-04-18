from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    
    fast_lio_dir = get_package_share_directory('fast_lio')
    slam_dir = get_package_share_directory('slam_toolbox')
    nav2_dir = get_package_share_directory('nav2_bringup')
    nav_control_hub_dir = os.path.join(
        os.path.expanduser("~"),
        'fyp_ws',
        'src',
        'HKU-2425FYP-RobotHelper',
        'navigation',
        'nav_control_hub'
    )
    
    nav_params_path = LaunchConfiguration(
        'nav_params', 
        default=os.path.join(nav_control_hub_dir, 'configs', 'nav_params.yaml')
    )
    slam_params_path = LaunchConfiguration(
        'slam_params', 
        default=os.path.join(nav_control_hub_dir, 'configs', 'slam_params.yaml')
    )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                fast_lio_dir,
                'launch',
                'mapping.launch.py'
            )
        ),
        launch_arguments={
            'config_file': 'avia.yaml', 
            'rviz': 'false'
        }.items()
    )
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                slam_dir,
                'launch',
                'online_async_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': slam_params_path,
        }.items()
    )
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav2_dir,
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav_params_path
        }.items()
    )
    
    control_node = Node(
        package='nav_control_hub',
        executable='control',
        name='control_node',
        output='screen'
    )
    
    yolo_explore_node = Node(
        package='nav_control_hub',
        executable='explore',
        name='yolo_explore',
        output='screen'
    )
    
    serial_node = Node(
        package='nav_control_hub',
        executable='serial',
        name='serial_node',
        output='screen'
    )
    
    return LaunchDescription([
        fast_lio_launch,
        slam_launch,
        nav2_launch,
        control_node,
        yolo_explore_node,
        serial_node
    ])

