import rclpy
from rclpy.node import Node
import rclpy.serialization
from realsense2_camera_msgs.msg import RGBD
from std_msgs.msg import String
from cv_bridge import CvBridge
import socket
import argparse
import threading

# Use realsense_rgbd.sh to open camera.

# RGBD Message
# std_msgs/Header header
# sensor_msgs/CameraInfo rgb_camera_info
# sensor_msgs/CameraInfo depth_camera_info
# sensor_msgs/Image rgb
# sensor_msgs/Image depth


class PromptNode(Node):
    def __init__(self, mode, namespace, camera_name):
        super().__init__("prompt_node")
        self.mode = mode
        self.namespace = namespace
        self.camera_name = camera_name
        self.bridge = CvBridge()
        self.rgbd_pub = self.create_publisher(RGBD, "/rgbd_remote", 10)
        self.prompt_pub = self.create_publisher(String, "/object_prompt", 10)
        self.get_logger().info(f"PromptNode initialized in {self.mode} mode")

        if self.mode == "local":
            self.local_mode()
        else:
            self.remote_mode()

    def local_mode(self):
        rgbd_topic = f"/{self.namespace}/{self.camera_name}/rgbd"
        self.rgbd_sub = self.create_subscription(RGBD, rgbd_topic, self.rgbd_callback, 10)
        threading.Thread(target=self.handle_user_input, daemon=True).start()

    def handle_user_input(self):
        while rclpy.ok():
            prompt = input("Enter prompt: ")
            prompt_msg = String()
            prompt_msg.data = prompt
            self.prompt_pub.publish(prompt_msg)

    def rgbd_callback(self, msg):
        # self.get_logger().info("RGBD images received")
        self.rgbd_pub.publish(msg)

    def remote_mode(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(("0.0.0.0", 30358))
        server_socket.listen(1)
        self.get_logger().info("Waiting for connection...")
        conn, addr = server_socket.accept()
        self.get_logger().info(f"Connected to {addr}")

        while rclpy.ok():
            data = conn.recv(4096)
            if not data:
                break

            rgbd_msg = rclpy.serialization.deserialize_message(data, RGBD)
            self.rgbd_pub.publish(rgbd_msg)

            prompt = conn.recv(1024).decode("utf-8")
            prompt_msg = String()
            prompt_msg.data = prompt
            self.prompt_pub.publish(prompt_msg)


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--local", action="store_true", help="Run in local mode")
    parser.add_argument("--namespace", type=str, default="grasp_module", help="Camera namespace")
    parser.add_argument("--camera", type=str, default="D435i", help="Camera name")
    args = parser.parse_args()

    mode = "local" if args.local else "remote"
    node = PromptNode(mode, args.namespace, args.camera)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Program interrupted. Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
