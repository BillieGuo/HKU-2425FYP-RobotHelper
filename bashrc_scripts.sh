source /home/robot-helper/ws_livox/install/setup.sh
source /home/robot-helper/fyp_ws/install/setup.sh
function arm_control() {
    ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=vx300s use_rviz:=false
}

function arm_manipulator(){
    ros2 run robotic_arm arm_manipulator
}

function realsense_up() {
    ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=640x480x30 enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true camera_namespace:=grasp_module camera_name:=D435i pointcloud.enable:=true
}

function grasp_request() {
    ros2 run robotic_arm grasp_request_node
}

function grasp_response() {
    ros2 run robotic_arm grasp_response_node
}

function grasp_prompt() {
    local prompt="$1"
    ros2 topic pub /grasp_prompt std_msgs/String "data: '$prompt'" --once
}

function view_angle() {
    local command="$1"
    local rate="$2"
    ros2 topic pub /robotic_arm/view_angle std_msgs/msg/String "{data: "$command"}" -r $rate
}

function navigation_control() {
    ros2 launch nav_control_hub navigator.launch.py
}

function livox_up() {
    ros2 launch livox_to_laserscan sensor_data.launch.py
}

function translator_ready() {
    ros2 launch translator translator.launch.py
}
