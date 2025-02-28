import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from realsense2_camera_msgs.msg import RGBD
from std_msgs.msg import String
from custom_msgs.msg import GraspQuery
import socket
import argparse


class ArmClient(Node):
    def __init__(self, camera_namespace, camera_name):
        super().__init__("arm_client")
        self.rgbd_msg = None
        self.sock = None
        rgbd_topic = f"/{camera_namespace}/{camera_name}/rgbd"
        self.rgbd_sub = self.create_subscription(RGBD, rgbd_topic, self.rgbd_callback, 10)
        self.prompt_sub = self.create_subscription(String, "/grasp_prompt", self.prompt_callback, 10)
        self.get_logger().info(f"ArmClient node initialized with camera namespace: {camera_namespace}, camera name: {camera_name}")
        self.connect_to_server()

    def connect_to_server(self):
        server_address = ("Carbonado", 30358)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(server_address)
        self.get_logger().info("Connected to server")

    def rgbd_callback(self, msg):
        self.rgbd_msg = msg

    def prompt_callback(self, msg):
        if self.rgbd_msg is None:
            self.get_logger().warn("RGBD message not received yet")
            return

        grasp_query = GraspQuery()
        grasp_query.text_prompt = msg.data
        grasp_query.rgbd_msg = self.rgbd_msg

        serialized_msg = serialize_message(grasp_query)
        # print(type(serialized_msg), len(serialized_msg))
        self.send_to_server(serialized_msg)

    def send_to_server(self, data):
        try:
            data_length = len(data)
            # Send the length of the data
            self.sock.sendall(data_length.to_bytes(4, byteorder='big'))
            # Send the actual data
            self.sock.sendall(data)
            self.get_logger().info("GraspQuery sent to server")
        except (BrokenPipeError, ConnectionResetError):
            self.get_logger().warn("Connection to server lost. Reconnecting...")
            self.connect_to_server()
            self.send_to_server(data)

    def destroy_node(self):
        if self.sock:
            self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", type=str, default="grasp_module", help="Camera namespace")
    parser.add_argument("--camera", type=str, default="D435i", help="Camera name")
    args = parser.parse_args()

    node = ArmClient(args.namespace, args.camera)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Program interrupted. Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
