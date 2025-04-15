import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from .gdino_sam import GDinoSAM
from datetime import datetime
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy

# RGBD Message
# std_msgs/Header header
# sensor_msgs/CameraInfo rgb_camera_info
# sensor_msgs/CameraInfo depth_camera_info
# sensor_msgs/Image rgb
# sensor_msgs/Image depth


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
        self.rgb_camera_info = None
        self.depth_camera_info = None
        self.ready_flag = False
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        # qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.rgbd_sub = self.create_subscription(RGBD, "/rgbd_remote", self.rgbd_callback, qos_profile)
        self.prompt_sub = self.create_subscription(String, "/object_prompt", self.prompt_callback, qos_profile)
        self.cropped_rgbd_pub = self.create_publisher(RGBD, "/cropped_rgbd", 10)
        self.debug_color_pub = self.create_publisher(Image, "/server_debug/cropped_color", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("The image_processor node initialized successfully, listening to the prompt\n")

    def rgbd_callback(self, msg: RGBD):
        self.get_logger().info("RGBD images received\n")
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8")
        self.depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "16UC1")
        self.rgb_camera_info = msg.rgb_camera_info
        self.depth_camera_info = msg.depth_camera_info
        if self.ready_flag is False:
            if self.prompt is not None:
                self.ready_flag = True

    def prompt_callback(self, msg):
        self.prompt = msg.data
        self.get_logger().info(f"Prompt '{self.prompt}' received")
        if self.ready_flag is False:
            if self.rgb_image is not None:
                self.ready_flag = True
    
    def timer_callback(self):
        if self.ready_flag is True:
            self.ready_flag = False
            self.get_logger().info("Start the inference\n")
            mask, absolute_boxes, logits, phrases = self.gdino_sam.infer(self.rgb_image, self.depth_image, self.prompt)
            if mask is None:
                self.get_logger().info("Object detection or segmentation failed. Publishing empty message.\n")
                self.prompt = None
                self.rgb_image = None
                self.depth_image = None
                rgbd_msg = RGBD()
                self.cropped_rgbd_pub.publish(rgbd_msg)
                return
            annotated_image = self.annotate_image(self.rgb_image, mask, absolute_boxes, logits, phrases)
            cropped_color, cropped_depth = self.crop_image(self.rgb_image, self.depth_image, mask)
            self.publish_cropped_images(cropped_color, cropped_depth)
            self.take_record(self.rgb_image, self.depth_image, cropped_color, cropped_depth, mask, annotated_image)
            self.get_logger().info("Image processing done\n")
            self.prompt = None

    def crop_image(self, color_image, depth_image, mask):
        masked_color = np.zeros_like(color_image)
        masked_depth = np.zeros_like(depth_image)
        masked_color[mask] = color_image[mask]
        masked_depth[mask] = depth_image[mask]
        return masked_color, masked_depth

    def publish_cropped_images(self, color_image, depth_image):
        rgbd_msg = RGBD()
        rgbd_msg.rgb = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
        rgbd_msg.depth = self.bridge.cv2_to_imgmsg(depth_image, "16UC1")
        rgbd_msg.rgb_camera_info = self.rgb_camera_info
        rgbd_msg.depth_camera_info = self.depth_camera_info
        self.cropped_rgbd_pub.publish(rgbd_msg)
        self.debug_color_pub.publish(rgbd_msg.rgb)
        self.get_logger().info("Cropped RGBD message published\n")

    def annotate_image(self, image, mask, absolute_boxes, logits, phrases):
        # Visualize the predicted box and mask on the color image
        color_mask = np.zeros_like(image, dtype=np.uint8)
        color = (0, 0, 255)
        color_mask[mask] = color
        # Blend original image with color mask
        alpha = 0.5
        blended_image = cv2.addWeighted(image, 1.0, color_mask, alpha, 0)
        # Draw the bounding box and label on the blended image
        bbox = absolute_boxes[0]
        x1, y1, x2, y2 = map(int, bbox)
        label = phrases[0] if phrases else self.prompt
        score = logits[0].item() if logits is not None else 0.0
        cv2.rectangle(blended_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(blended_image, f"{label}: {score:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        return blended_image

    def take_record(self, rgb_image, depth_image, cropped_color, cropped_depth, mask, annotated_image):
        # Save these information to the record_dir/record_{timestamp}/ separately
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        record_dir = os.path.join(self.records, f"record_{timestamp}")
        os.makedirs(record_dir, exist_ok=True)

        # Save the images
        cv2.imwrite(os.path.join(record_dir, "rgb_image.png"), rgb_image)
        cv2.imwrite(os.path.join(record_dir, "depth_image.png"), depth_image)
        cv2.imwrite(os.path.join(record_dir, "annotated_image.png"), annotated_image)
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
