#!/usr/bin/env python3
from PIL import Image
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from realsense2_camera_msgs.msg import RGBD  # Import the RGBD message type
from std_msgs.msg import String
from cv_bridge import CvBridge
import threading
import readchar  # Replaced keyboard with readchar

os.environ["TORCH_CUDA_VERBOSE"] = "1"

save_dir = "/home/intern/fyp_ws/src/HKU-2425FYP-RobotHelper/grasp/gdino_sam/assets/realsense_ros_test"
os.makedirs(save_dir, exist_ok=True)
# RGBD Message
# std_msgs/Header header
# sensor_msgs/CameraInfo rgb_camera_info
# sensor_msgs/CameraInfo depth_camera_info
# sensor_msgs/Image rgb
# sensor_msgs/Image depth


class realsense_test_node(Node):
    def __init__(self, camera_namespace, camera_name):
        super().__init__("realsense_test_node")

        # Subscribe both color and depth images
        rgbd_topic = f"/{camera_namespace}/{camera_name}/rgbd"
        self.rgbd_sub = self.create_subscription(RGBD, rgbd_topic, self.rgbd_callback, 10)

        self.bridge = CvBridge()
        self.latest_color_image = None
        self.latest_depth_image = None
        self.save_image = False  # Flag to trigger image saving

        # Start keyboard listener thread
        threading.Thread(target=self.listen_keyboard, daemon=True).start()

        self.get_logger().info(
            f"Realsense test node has been started.\nNamespace/Name: {camera_namespace}/{camera_name}\nPress q to quit, s to save images."
        )

    def listen_keyboard(self):
        while rclpy.ok():
            char = readchar.readchar()
            if char == "s":
                self.save_image = True
                self.get_logger().info("Save command received.")
            elif char == "q":
                self.get_logger().info("Quit command received.")
                rclpy.shutdown()

    def rgbd_callback(self, msg):
        self.latest_color_image = self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8")
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "16UC1")
        # print(f"Color image shape: {self.latest_color_image.shape}")
        # Image saving functionality triggered by key press
        if self.save_image and self.latest_color_image is not None and self.latest_depth_image is not None:
            color_path = os.path.join(save_dir, "saved_color.png")
            depth_path = os.path.join(save_dir, "saved_depth.png")
            cv2.imwrite(color_path, self.latest_color_image)
            cv2.imwrite(depth_path, self.latest_depth_image)
            self.get_logger().info(f"Images saved to {color_path} and {depth_path}.")
            self.save_image = False


def main(args=None):
    rclpy.init(args=args)
    node = realsense_test_node(camera_namespace="grasp_module", camera_name="D435i")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
