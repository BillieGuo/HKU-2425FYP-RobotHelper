#!/usr/bin/env python3
from PIL import Image
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import threading
import readchar  # Replaced keyboard with readchar

os.environ["TORCH_CUDA_VERBOSE"] = "1"

script_dir = os.path.dirname(os.path.abspath(__file__))


class realsense_test_node(Node):
    def __init__(self, camera_namespace, camera_name):
        super().__init__("realsense_test_node")

        # Subscribe both color and depth images
        rgbd_topic = f"/{camera_namespace}/{camera_name}/rgbd"
        self.rgbd_sub = self.create_subscription(ROSImage, rgbd_topic, self.rgbd_callback, 10)

        self.bridge = CvBridge()
        self.latest_color_image = None
        self.latest_depth_image = None
        self.save_image = False  # Flag to trigger image saving

        # Start keyboard listener thread
        threading.Thread(target=self.listen_keyboard, daemon=True).start()

        self.get_logger().info("Realsense test node has been started.")

    def listen_keyboard(self):
        while rclpy.ok():
            char = readchar.readchar()
            if char == "s":
                self.save_image = True
                self.get_logger().info("Save command received.")

    def rgbd_callback(self, msg):
        self.latest_color_image = self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8")
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "16UC1")
        print(f"Color image shape: {self.latest_color_image.shape}")
        # Image saving functionality triggered by key press
        if self.save_image and self.latest_color_image is not None and self.latest_depth_image is not None:
            color_path = os.path.join(script_dir, "saved_color.png")
            depth_path = os.path.join(script_dir, "saved_depth.png")
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
        node.get_logger().info("Program interrupted. Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
