import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import os
import yaml
import numpy as np
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from enum import Enum
from tf_transformations import quaternion_from_matrix

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
        self.broadcast_c2g_tf()

        self.state = ArmState.CAPTURING
    
    def broadcast_c2g_tf(self):
        tf_file_path =  os.path.join(os.path.expanduser("~/fyp_ws/src/HKU-2425FYP-RobotHelper/grasp/robotic_arm/robotic_arm/config"), "transform_camera2gripper.yaml")
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