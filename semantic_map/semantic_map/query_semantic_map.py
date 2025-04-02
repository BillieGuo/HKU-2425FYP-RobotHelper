import socket
import rclpy
import rclpy.serialization
from rclpy.node import Node
import struct
import time

import rclpy
import numpy as np

from custom_msgs.msg import ObjectSem

class SocketSender(Node):
    def __init__(self, connect_to='fyp'):
        super().__init__('image_subscriber_socket_sender')

        self.connect_to = connect_to
        self.socket_setup()
    
    def receive_semantic_map_results(self):
        num_of_maps = struct.unpack('<L', self.sock.recv(4))[0]
        self.get_logger().info(f"Number of maps: {num_of_maps}")
        list_of_results = []
        for i in range(num_of_maps):
            data_size = struct.unpack('<L', self.sock.recv(4))[0]
            print("data_size", data_size)
            data = b""
            while len(data) < data_size:
                packet = self.sock.recv(data_size - len(data))
                if not packet:
                    break
                data+=packet
            list_of_results.append(rclpy.serialization.deserialize_message(data, ObjectSem))
            print(list_of_results)
            self.send_text("send_next")
        return list_of_results
    
    def send_text(self, text):
        try:
            self.get_logger().info(f"Sending text: {text}")
            data = text.encode()
            self.sock.sendall(data)
        except Exception as e:
            self.get_logger().error(f"Failed to send text: {e}")
    
    def send_object_name(self, object_name):
        try:
            self.get_logger().info(f"Sending object name: {object_name}")
            data = object_name.encode()
            self.sock.sendall(struct.pack('<L',len(data)) + data)
        except Exception as e:
            self.get_logger().error(f"Failed to send object name: {e}")

    def send_double(self, float_threshold):
        try:
            self.get_logger().info(f"Sending double threshold: {float_threshold}")
            data = struct.pack('d', float_threshold)
            self.sock.sendall(data)
        except Exception as e:
            self.get_logger().error(f"Failed to send double threshold: {e}")

    def socket_setup(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.connect_to, 8896))
    
    def wait_handshake(self, handshake_msg="handshake"):
        self.get_logger().info(f"Waiting for handshake")
        handshake = self.sock.recv(1024).decode()
        if handshake != handshake_msg:
            self.get_logger().error("Handshake failed")
            self.get_logger().error(f"Received: {handshake}, expect: {handshake_msg}")
            return False
        else:
            self.get_logger().info("Handshake successful")
            return True

    def close_socket(self):
        self.sock.close()

def main(args=None):
    rclpy.init(args=args)
    socket_sender = SocketSender(connect_to='fyp')
    print('connected')
    try:
        while rclpy.ok():
            print("start send")
            socket_sender.send_text("query")
            print("waiting")
            socket_sender.wait_handshake("send_object")
            socket_sender.send_object_name("keyboard")
            socket_sender.wait_handshake("send_sim")
            socket_sender.send_double(0.5)
            list_of_results = socket_sender.receive_semantic_map_results()
            print(list_of_results)
            socket_sender.wait_handshake("done")
            time.sleep(1)

    except KeyboardInterrupt:
        pass
    finally:
        socket_sender.close_socket()
        socket_sender.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()