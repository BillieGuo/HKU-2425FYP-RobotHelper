import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from custom_msgs.msg import GraspResponse
import os
import yaml
import numpy as np
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from enum import Enum
from tf_transformations import quaternion_from_matrix
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory("robotic_arm")

# Test 1: Check the Camera optical frame and the Gripper frame

class ArmState(Enum):
    CAPTURING = 1
    GRASPING = 2

class ArmManipulator(InterbotixManipulatorXS):
    def __init__(self):
        super().__init__(
            robot_model='vx300s',
            group_name='arm',
            gripper_name='gripper',
        )
        self.node = self.core.get_node()
        # c2g is camera to gripper transform
        self.c2g_broadcaster = StaticTransformBroadcaster(self.node)
        self.grasp_pose_broadcaster = TransformBroadcaster(self.node)
        self.broadcast_c2g_tf()
        self.create_publisher()
        self.create_subscriber()
        # self.READY_POSITIONS = [0, -1.1, 1.1, 0, 0.9, 0]
        self.CAPTURE_VIEW_JOINTS = [-0.5, -1.1, 0.4, 0, 1.6, 0]

        self.state = ArmState.CAPTURING
    
    def broadcast_c2g_tf(self):
        tf_file_path =  os.path.join(package_share_directory, "config", "transform_camera2gripper.yaml")
        with open(tf_file_path, "r") as file:
            c2g_file = yaml.safe_load(file)
        c2g_matrix = np.array(c2g_file["camera2gripper"])
        c2g_tf = TransformStamped()
        c2g_tf.header.stamp = self.node.get_clock().now().to_msg()
        c2g_tf.header.frame_id = "vx300s/ee_gripper_link"
        c2g_tf.child_frame_id = "camera_optical_link"
        c2g_tf.transform.translation.x = c2g_matrix[0][3]
        c2g_tf.transform.translation.y = c2g_matrix[1][3]
        c2g_tf.transform.translation.z = c2g_matrix[2][3]
        c2g_tf.transform.rotation.x, c2g_tf.transform.rotation.y, c2g_tf.transform.rotation.z, c2g_tf.transform.rotation.w = quaternion_from_matrix(c2g_matrix)
        self.c2g_broadcaster.sendTransform(c2g_tf)


    def create_publisher(self):
        # Publish for the grasp request node
        self.prompt_pub = self.node.create_publisher(String, "/text_prompt", 10)

    def create_subscriber(self):
        # translator's prompt
        self.prompt_sub = self.node.create_subscription(String, "/grasp_prompt", self.prompt_callback, 10)
        # grasp poses results
        self.grasp_sub = self.node.create_subscription(GraspResponse, "/robotic_arm/grasp_response", self.grasp_callback, 10)

    def prompt_callback(self, msg):
        # Receive the prompt from the LLM translator
        self.node.get_logger().info(f"Received prompt: {msg.data}")
        self.text_prompt = msg.data
        self.node.get_logger().info("Release the gripper")
        self.release()
        self.node.get_logger().info("Going to capture pose")
        self.go_to_capture_pose()
        self.prompt_pub.publish(msg)
        self.node.get_logger().info("Publish the prompt, waiting for the grasp pose")

    def grasp_callback(self, msg):
        # Receive the grasp pose from the server
        self.node.get_logger().info(f"Received grasp pose: {msg}")
        self.num_grasp_poses = msg.num_grasp_poses
        self.grasp_poses = msg.grasp_poses
        self.scores = msg.scores
        # Use the best grasp pose
        grasp_pose = self.grasp_poses[0]
        grasp_pose_tf_msg = TransformStamped()
        grasp_pose_tf_msg.header.stamp = self.node.get_clock().now().to_msg()
        grasp_pose_tf_msg.header.frame_id = "camera_optical_link"
        grasp_pose_tf_msg.child_frame_id = "grasp_pose"
        grasp_pose_tf_msg.transform.translation.x = grasp_pose.position.x
        grasp_pose_tf_msg.transform.translation.y = grasp_pose.position.y
        grasp_pose_tf_msg.transform.translation.z = grasp_pose.position.z
        grasp_pose_tf_msg.transform.rotation.x = grasp_pose.orientation.x
        grasp_pose_tf_msg.transform.rotation.y = grasp_pose.orientation.y
        grasp_pose_tf_msg.transform.rotation.z = grasp_pose.orientation.z
        grasp_pose_tf_msg.transform.rotation.w = grasp_pose.orientation.w
        self.grasp_pose_broadcaster.sendTransform(grasp_pose_tf_msg)
        self.node.get_logger().info("Broadcast the grasp pose")
        # self.grasp()
        


    def go_to_capture_pose(self):
        self.arm.set_joint_positions(self.CAPTURE_VIEW_JOINTS, moving_time=5.0)

    def grasp(self):
        # PWM
        self.gripper.grasp()

    def release(self):
        # PWM
        self.gripper.release()

    def run(self):
        try:
            robot_startup()
            while rclpy.ok():
                self.handle_state()
                
        except KeyboardInterrupt:
            robot_shutdown()

    def handle_state(self):
        if self.state == ArmState.CAPTURING:
            self.capture()
        elif self.state == ArmState.GRASPING:
            self.grasp()

    def capture(self):
        # Capturing logic
        # Wait for a specific command to transition to the next state
        pass

    def grasp(self):
        # Grasping logic
        # Move the arm and the gripper to grasp an object
        pass

def main(args=None):
    rclpy.init(args=args)
    robotic_arm = ArmManipulator()
    robotic_arm.run()

if __name__ == "__main__":
    main()