import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from custom_msgs.msg import GraspResponse
import socket


class GraspExecuteNode(Node):
    def __init__(self, camera_namespace, camera_name):
        super().__init__("grasp_execute_node")
        self.sock = None
        self.responder_sock = None
        self.get_logger().info(f"Grasp execute node initialized.")
        self.connect_to_responder()

    def connect_to_responder(self):
        responder_address = ("Carbonado", 45851)
        self.responder_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.responder_sock.connect(responder_address)
        self.get_logger().info("Connected to responder server")

    def receive_from_responder(self):
        try:
            # Receive the length of the data
            data_length = int.from_bytes(self.responder_sock.recv(4), byteorder='big')
            # Receive the actual data
            data = self.responder_sock.recv(data_length)
            grasp_response = deserialize_message(data, GraspResponse)
            self.grasp_response_pub.publish(grasp_response)
            self.get_logger().info("GraspResponse received from responder and published")
        except (BrokenPipeError, ConnectionResetError):
            self.get_logger().warn("Connection to responder server lost. Reconnecting...")
            self.connect_to_responder()
            self.receive_from_responder()

    def destroy_node(self):
        if self.sock:
            self.sock.close()
        if self.responder_sock:
            self.responder_sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GraspExecuteNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Program interrupted. Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
