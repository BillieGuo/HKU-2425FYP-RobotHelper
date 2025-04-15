import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from custom_msgs.msg import GraspResponse
import socket
import threading

class GraspResponder(Node):
    def __init__(self):
        super().__init__("grasp_responder")
        self.sock = None
        self.connection = None
        self.grasp_response_sub = self.create_subscription(GraspResponse, "/grasp_response", self.grasp_response_callback, 10)
        threading.Thread(target=self.connect_to_client, daemon=True).start()
        self.get_logger().info("GraspResponder node initialized and listening to /grasp_response")

    def connect_to_client(self):
        while rclpy.ok():
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.bind(("0.0.0.0", 45851))
            self.sock.listen(1)
            self.get_logger().info("Waiting for connection from arm client...")
            self.connection, self.client_address = self.sock.accept()
            self.get_logger().info("Connected to arm client")

            while rclpy.ok():
                try:
                    # Keep the connection alive
                    self.connection.recv(1)
                except (ConnectionResetError, BrokenPipeError):
                    self.get_logger().warn("Connection to arm client lost. Reconnecting...")
                    break

            self.connection.close()
            self.sock.close()

    def grasp_response_callback(self, msg):
        if self.connection is None:
            return
        try:
            serialized_msg = serialize_message(msg)
            data_length = len(serialized_msg)
            # Send the length of the data
            self.connection.sendall(data_length.to_bytes(4, byteorder='big'))
            # Send the actual data
            self.connection.sendall(serialized_msg)
            self.get_logger().info("GraspResponse sent to arm client")
        except (BrokenPipeError, ConnectionResetError):
            self.get_logger().warn("Connection to arm client lost. Reconnecting...")
            threading.Thread(target=self.connect_to_client, daemon=True).start()

    def destroy_node(self):
        if self.connection:
            self.connection.close()
        if self.sock:
            self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GraspResponder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Program interrupted. Exiting...")
        node.connection.close()
        node.sock.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
