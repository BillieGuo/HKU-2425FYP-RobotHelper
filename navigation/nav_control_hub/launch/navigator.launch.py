from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fast_lio'),
                'launch',
                'mapping.launch.py'
            ])
        ),
        launch_arguments={
            'config_file': 'avia.yaml', 
            'rviz': 'false'
        }.items()
    )
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'false'
        }.items()
    )
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'false'
        }.items()
    )
    
    control_node = Node(
        package='nav_control_hub',
        executable='control_node',
        name='control_node',
        output='screen'
    )
    
    yolo_explore_node = Node(
        package='nav_control_hub',
        executable='yolo_explore',
        name='yolo_explore',
        output='screen'
    )
    
    serial_node = Node(
        package='nav_control_hub',
        executable='serial_node',
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
