import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2
import yaml
from .gdino_sam import GDinoSAM


class ImageProcessor(Node):
    def __init__(self):
        super().__init__("image_processor")
        self.declare_parameter("config_file", "config/image_process_config.yaml")
        config_file = self.get_parameter("config_file").get_parameter_value().string_value
        with open(config_file, "r") as file:
            self.config = yaml.safe_load(file)

        self.gdino_sam = GDinoSAM(
            groundingdino_weights_path=self.config["paths"]["groundingdino_weights_path"],
            groundingdino_config_path=self.config["paths"]["groundingdino_config_path"],
            sam_weights_path=self.config["paths"]["sam_weights_path"],
            mask_storage_path=self.config["paths"]["mask_storage_path"],
            box_threshold=self.config["thresholds"]["box_threshold"],
            text_threshold=self.config["thresholds"]["text_threshold"],
        )
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.prompt = None

        self.rgbd_sub = self.create_subscription(ROSImage, "/rgbd_remote", self.rgbd_callback, 10)
        self.prompt_sub = self.create_subscription(String, "/object_prompt", self.prompt_callback, 10)
        self.cropped_image_pub = self.create_publisher(ROSImage, "cropped_image", 10)

    def rgbd_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8")
        self.depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "16UC1")

    def prompt_callback(self, msg):
        self.prompt = msg.data
        if self.rgb_image is not None and self.depth_image is not None:
            mask = self.gdino_sam.infer(self.rgb_image, self.depth_image, self.prompt)
            cropped_color, cropped_depth = self.crop_image(self.rgb_image, self.depth_image, mask)
            self.publish_cropped_images(cropped_color, cropped_depth)

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
