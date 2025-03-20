from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'enable_rgbd': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'camera_namespace': 'grasp_module',
            'camera_name': 'D435i'
        }.items()
    )

    robotic_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            )
        ),
        launch_arguments={
            'robot_model': 'vx300s'
        }.items()
    )

    arm_manipulator_node = Node(
        package='robotic_arm',
        executable='arm_manipulator',
        name='arm_manipulator'
    )

    grasp_request_node = Node(
        package='robotic_arm',
        executable='grasp_request_node',
        name='grasp_request_node'
    )

    grasp_response_node = Node(
        package='robotic_arm',
        executable='grasp_response_node',
        name='grasp_response_node'
    )

    return LaunchDescription([
        realsense_launch,
        robotic_arm_launch,
        arm_manipulator_node,
        grasp_request_node,
        grasp_response_node
    ])
