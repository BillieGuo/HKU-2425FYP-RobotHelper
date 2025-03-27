import socket
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SocketServer(Node):
    def __init__(self, host='robot-helper', port=7000):
        super().__init__("socket")
        self.llm_response = ""
        
        # socket
        self.host = host
        self.port = port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((self.host, self.port))
        self.server.listen(5)
        self.get_logger().info(f"Server listening on {self.host}:{self.port}")
        
        # ros
        self.socket2llm_pub = self.create_publisher(
            String,
            'socket2llm',
            10)
        self.llm2socket_sub = self.create_subscription(
            String,
            'llm2socket',
            self.llm_response_callback,
            10)
        
    def llm_response_callback(self, msg):
        self.llm_response = msg.data
        pass 
        
    def handle_client(self, client_socket):
        while True:
            data = client_socket.recv(1024)
            if not data:
                break
            # client_socket.send(data)  # temp placer
            data = data.decode()
            self.get_logger().info(f"Received: {data}")
            ros_msg = String()
            ros_msg.data = data
            self.socket2llm_pub.publish(ros_msg)
            # block and wait for llm response then send back to client
            rclpy.spin_once(self)
            client_socket.send(self.llm_response.encode())  # Echo the received message back to the client
            self.llm_response = ""
        client_socket.close()

    def start(self):
        while True:
            client_socket, addr = self.server.accept()
            self.get_logger().info(f"Connection from {addr}")
            self.handle_client(client_socket)
            
    def __del__(self):
        self.server.close()
            
def main():
    rclpy.init()
    node = SocketServer()
    node.start()
    node.server.close()
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()