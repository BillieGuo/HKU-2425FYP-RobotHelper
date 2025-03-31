'''
    This is a simple socket client that sends a message to the server and waits for a response.
    It shall be used to communicate with the Semantic Map server, and can be one another machine.
'''

import socket
import struct
import time
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy


class SocketClient(Node):
    def __init__(self, host='robot-helper', port=7000):
        super().__init__("socket_client2SemanticMap")        
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.navigator2socket_sub = self.create_subscription(
            String,
            'navigator2socket',
            self.navigator_request_callback,
            qos_profile)
        self.socket2navigator_pub = self.create_publisher(
            String,
            'socket2navigator',
            qos_profile)
        
        self.host = host
        self.port = port
        self.client = None
        self.connect()
        
        self.prompt = None

    def navigator_request_callback(self, msg):
        if msg.data:
            self.get_logger().info(f"Received from navigator: {msg.data}")
            self.prompt = msg.data
            
    def connect(self):
        try:
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client.connect((self.host, self.port))
            return True
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, ConnectionRefusedError):
            print('Server is down, try to reconnect')
            return False
        
    def send(self, msg):
        if not hasattr(self, 'client') or self.client.fileno() == -1:
            success = self.connect()
            while not success:
                success = self.connect()
                
        try:
            self.client.send(msg.encode())
            self.get_logger().info(f"Sent: {msg.data}")
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, ConnectionRefusedError):
            print('Server is down, try to reconnect')
            self.connect()
            return
        data = self.client.recv(12)
        self.get_logger().info(f"Received from Semantic Map server: {data}")
        return np.array(struct.unpack('<3f', data))

    def pub2navigator(self, msg):
        tx = String()
        tx.data = msg
        self.socket2navigator_pub.publish(tx)

    def run(self):
        success = self.connect()
        while not success:
            success = self.connect()
            
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.prompt is None:
                continue
            
            response = self.send(self.prompt)
            self.pub2navigator(f"[{response[0]},{response[1]},{response[2]}]")
            self.prompt = None
            
            
    def __del__(self):
        self.client.close()

def main():
    rclpy.init()
    node = SocketClient(host='fyp', port=6000)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.client.close()
    node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()