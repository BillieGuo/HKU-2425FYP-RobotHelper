import rclpy
from rclpy.node import Node
import rclpy.serialization
from realsense2_camera_msgs.msg import RGBD
from std_msgs.msg import String
from cv_bridge import CvBridge
import socket
import argparse
import threading
from custom_msgs.msg import GraspQuery


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

    def remote_mode(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(("0.0.0.0", 30358))
        server_socket.listen(1)
        self.get_logger().info("Waiting for connection...")

        while rclpy.ok():
            conn, addr = server_socket.accept()
            self.get_logger().info(f"Connected to {addr}")

            while rclpy.ok():
                try:
                    # Receive the serialized query message as the bytearray
                    data_length_bytes = conn.recv(4)
                    if not data_length_bytes:
                        break
                    data_length = int.from_bytes(data_length_bytes, byteorder="big")
                    data = bytearray()
                    while len(data) < data_length:
                        packet = conn.recv(data_length - len(data))
                        if not packet:
                            break
                        data.extend(packet)
                    if not data:
                        break
                    # Convert bytearray to bytes
                    data_bytes = bytes(data)
                    # Deserialize the message
                    grasp_query = rclpy.serialization.deserialize_message(data_bytes, GraspQuery)
                    # Get and publish the RGBD message and the prompt string message for image processor
                    prompt_msg = String()
                    prompt_msg.data = grasp_query.text_prompt
                    rgbd_msg = grasp_query.rgbd_msg
                    self.get_logger().info(f"The grasp query received, with the text prompt {prompt_msg.data}.")
                    self.rgbd_pub.publish(rgbd_msg)
                    self.prompt_pub.publish(prompt_msg)
                    self.get_logger().info("RGBD images and prompt message published to /rgbd_remote and /object_prompt topics.")
                except (ConnectionResetError, BrokenPipeError):
                    self.get_logger().warn("Client disconnected. Waiting for new connection...")
                    break

            conn.close()

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


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--local", action="store_true", help="Run in local mode")
    parser.add_argument("--namespace", type=str, default="grasp_module", help="Camera namespace")
    parser.add_argument("--camera", type=str, default="D435i", help="Camera name")
    args, unknown = parser.parse_known_args(rclpy.utilities.remove_ros_args(args))

    mode = "local" if args.local else "remote"
    # mode = "local"
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
