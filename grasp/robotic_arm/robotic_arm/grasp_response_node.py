import rclpy
from rclpy.node import Node
import socket
from rclpy.serialization import deserialize_message
from custom_msgs.msg import GraspResponse
import threading

class GraspResponseNode(Node):
    def __init__(self):
        super().__init__("grasp_response_node")
        self.responder_sock = None
        self.grasp_response_pub = self.create_publisher(GraspResponse, "/robotic_arm/grasp_response", 10)
        self.connect_to_responder()
        threading.Thread(target=self.listen_to_responder, daemon=True).start()

    def connect_to_responder(self):
        responder_address = ("Carbonado", 45851)
        while not self.responder_sock:
            try:
                self.responder_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.responder_sock.connect(responder_address)
                self.get_logger().info("Connected to responder server")
            except (ConnectionRefusedError, OSError):
                self.get_logger().warn("Connection failed, retrying...")
                self.responder_sock = None
                rclpy.sleep(1)

    def listen_to_responder(self):
        while rclpy.ok():
            grasp_response = self.receive_from_responder()
            if grasp_response:
                self.grasp_response_pub.publish(grasp_response)
                self.get_logger().info("Grasp response published")

    def receive_from_responder(self):
        try:
            data_length = int.from_bytes(self.responder_sock.recv(4), byteorder='big')
            data = self.responder_sock.recv(data_length)
            grasp_response = deserialize_message(data, GraspResponse)
            return grasp_response
        except (BrokenPipeError, ConnectionResetError, OSError):
            self.get_logger().warn("Connection lost, reconnecting...")
            self.responder_sock.close()
            self.responder_sock = None
            self.connect_to_responder()
            return None

    def destroy_node(self):
        if self.responder_sock:
            self.responder_sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GraspResponseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Program interrupted. Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
