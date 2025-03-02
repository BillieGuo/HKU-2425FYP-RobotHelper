import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from .gdino_sam import GDinoSAM
from datetime import datetime
import cv2


class ImageProcessor(Node):
    def __init__(self):
        super().__init__("image_processor")
        package_share_directory = get_package_share_directory("gdino_sam")
        config_file_path = os.path.join(package_share_directory, "config", "image_process_config.yaml")
        with open(config_file_path, "r") as file:
            self.config = yaml.safe_load(file)
        self.records = os.path.join(self.config["paths"]["package_path"], self.config["paths"]["record_path"])

        self.get_logger().info(f"\nInitializing GDinoSAM using config file: \n{self.config}\n")
        self.gdino_sam = GDinoSAM(
            package_path=self.config["paths"]["package_path"],
            gdino_weights_path=self.config["paths"]["groundingdino_weights_path"],
            gdino_config_path=self.config["paths"]["groundingdino_config_path"],
            sam_weights_path=self.config["paths"]["sam_weights_path"],
            box_threshold=self.config["thresholds"]["box_threshold"],
            text_threshold=self.config["thresholds"]["text_threshold"],
        )
        self.get_logger().info("GDinoSAM initialized successfully\n")

        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.prompt = None

        self.rgbd_sub = self.create_subscription(RGBD, "/rgbd_remote", self.rgbd_callback, 10)
        self.prompt_sub = self.create_subscription(String, "/object_prompt", self.prompt_callback, 10)
        self.cropped_image_pub = self.create_publisher(ROSImage, "cropped_image", 10)
        self.get_logger().info("The image_processor node initialized successfully, listening to the prompt\n")

    def rgbd_callback(self, msg):
        # self.get_logger().info("RGBD images received\n")
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8")
        self.depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "16UC1")

    def prompt_callback(self, msg):
        self.prompt = msg.data
        self.get_logger().info(f"Prompt '{self.prompt}' received\n")
        if self.rgb_image is not None and self.depth_image is not None:
            self.get_logger().info("Start the inference\n")
            mask = self.gdino_sam.infer(self.rgb_image, self.depth_image, self.prompt)
            cropped_color, cropped_depth = self.crop_image(self.rgb_image, self.depth_image, mask)
            self.publish_cropped_images(cropped_color, cropped_depth)
            self.take_record(self.rgb_image, self.depth_image, cropped_color, cropped_depth, mask)
            self.prompt = None

    def crop_image(self, color_image, depth_image, mask):
        masked_color = np.zeros_like(color_image)
        masked_depth = np.zeros_like(depth_image)
        masked_color[mask] = color_image[mask]
        masked_depth[mask] = depth_image[mask]
        return masked_color, masked_depth

    def publish_cropped_images(self, color_image, depth_image):
        color_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, "16UC1")
        self.cropped_image_pub.publish(color_msg)
        self.cropped_image_pub.publish(depth_msg)
        self.get_logger().info("Cropped images published\n")

    def take_record(self, rgb_image, depth_image, cropped_color, cropped_depth, mask):
        # Save these information to the record_dir/record_{timestamp}/ separately
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        record_dir = os.path.join(self.records, f"record_{timestamp}")
        os.makedirs(record_dir, exist_ok=True)

        # Save the images
        cv2.imwrite(os.path.join(record_dir, "rgb_image.png"), rgb_image)
        cv2.imwrite(os.path.join(record_dir, "depth_image.png"), depth_image)
        cv2.imwrite(os.path.join(record_dir, "cropped_color.png"), cropped_color)
        cv2.imwrite(os.path.join(record_dir, "cropped_depth.png"), cropped_depth)
        cv2.imwrite(os.path.join(record_dir, "mask.png"), mask.astype(np.uint8) * 255)
        self.get_logger().info(f"Record at {record_dir}\n")


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Program interrupted. Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
