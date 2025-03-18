import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


class FakeCamera(Node):
    def __init__(self):
        super().__init__("fake_camera")
        self.rgbd_pub = self.create_publisher(RGBD, "/cropped_rgbd", 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(5.0, self.timer_callback)  # Publish every second
        self.get_logger().info("FakeCamera node has been started.")

    def timer_callback(self):
        # Load color and depth images
        color_image = cv2.imread("./cropped_color_mouse.png")
        depth_image = cv2.imread("./cropped_depth_mouse.png", cv2.IMREAD_UNCHANGED)

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
        # Hardcoded RGB camera info
        rgbd_msg.rgb_camera_info = CameraInfo()
        rgbd_msg.rgb_camera_info.header.frame_id = "D435i_color_optical_frame"
        p480 = False
        if p480:
            rgbd_msg.rgb_camera_info.height = 480
            rgbd_msg.rgb_camera_info.width = 640
            rgbd_msg.rgb_camera_info.distortion_model = "plumb_bob"
            rgbd_msg.rgb_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            rgbd_msg.rgb_camera_info.k = [
                606.4698486328125,
                0.0,
                325.1153259277344,
                0.0,
                606.5099487304688,
                239.0905303955078,
                0.0,
                0.0,
                1.0,
            ]
            rgbd_msg.rgb_camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            rgbd_msg.rgb_camera_info.p = [
                606.4698486328125,
                0.0,
                325.1153259277344,
                0.0,
                0.0,
                606.5099487304688,
                239.0905303955078,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ]
        else:
            rgbd_msg.rgb_camera_info.height = 720
            rgbd_msg.rgb_camera_info.width = 1280
            rgbd_msg.rgb_camera_info.distortion_model = "plumb_bob"
            rgbd_msg.rgb_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            rgbd_msg.rgb_camera_info.k = [
                909.7047119140625,
                0.0,
                647.6729736328125,
                0.0,
                909.7649536132812,
                358.6357727050781,
                0.0,
                0.0,
                1.0,
            ]
            rgbd_msg.rgb_camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            rgbd_msg.rgb_camera_info.p = [
                909.7047119140625,
                0.0,
                647.6729736328125,
                0.0,
                0.0,
                909.7649536132812,
                358.6357727050781,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ]
        rgbd_msg.rgb_camera_info.binning_x = 0
        rgbd_msg.rgb_camera_info.binning_y = 0
        rgbd_msg.rgb_camera_info.roi.x_offset = 0
        rgbd_msg.rgb_camera_info.roi.y_offset = 0
        rgbd_msg.rgb_camera_info.roi.height = 0
        rgbd_msg.rgb_camera_info.roi.width = 0
        rgbd_msg.rgb_camera_info.roi.do_rectify = False
        rgbd_msg.depth_camera_info = rgbd_msg.rgb_camera_info

        self.rgbd_pub.publish(rgbd_msg)
        self.get_logger().info("Published cropped RGBD message.")


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
