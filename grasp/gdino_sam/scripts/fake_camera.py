import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


class FakeCamera(Node):
    def __init__(self):
        super().__init__("fake_camera")
        self.rgbd_pub = self.create_publisher(RGBD, "/grasp_module/D435i/rgbd", 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0, self.timer_callback)  # Publish every second
        self.get_logger().info("FakeCamera node has been started.")

    def timer_callback(self):
        # Load color and depth images
        color_image = cv2.imread("./rgb_image_20241204_181629.png")
        depth_image = cv2.imread("./depth_image_20241204_181629.png", cv2.IMREAD_UNCHANGED)

        if color_image is None or depth_image is None:
            self.get_logger().error("Failed to load images.")
            return

        # Convert images to ROS messages
        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="mono16")

        # Create and publish RGBD message
        rgbd_msg = RGBD()
        rgbd_msg.rgb = color_msg
        rgbd_msg.depth = depth_msg
        rgbd_msg.rgb_camera_info = CameraInfo()
        rgbd_msg.depth_camera_info = CameraInfo()

        self.rgbd_pub.publish(rgbd_msg)
        self.get_logger().info("Published RGBD message.")


def main(args=None):
    rclpy.init(args=args)
    node = FakeCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Program interrupted. Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
