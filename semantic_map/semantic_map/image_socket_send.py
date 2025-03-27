import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import struct
import time

import tf2_ros
import rclpy
from geometry_msgs.msg import TransformStamped
import tf_transformations
import numpy as np

class SocketSender(Node):
    def __init__(self, camera_frame="camera_link", world_frame="map", connect_to='fyp'):
        super().__init__('image_subscriber_socket_sender')

        self.connect_to = connect_to
        self.image_sub = self.create_subscription(
            Image,
            '/grasp_module/D435i/color/image_raw',
            self.image_callback,
            10)
        self.depth_sub = self.create_subscription(
            Image, 
            '/grasp_module/D435i/aligned_depth_to_color/image_raw', 
            self.depth_callback, 
            10)
        
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.socket_setup()

        self.camera_frame = camera_frame
        self.world_frame = world_frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_to_world = None
    
    def wait_for_first_images(self):
        while self.color_image is None or self.depth_image is None:
            rclpy.spin_once(self)
            time.sleep(0.1)

    def socket_setup(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.connect_to, 8812))

    def listen_tf(self):
        try:
            self.camera_to_world = self.tf_buffer.lookup_transform(self.camera_frame, self.world_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info('Cannot find camera to world transform')
            return False
        return True    
    
    def send_transform(self):
        if self.camera_to_world is not None:
            translation = np.array([self.camera_to_world.transform.translation.x,
                                    self.camera_to_world.transform.translation.y,
                                    self.camera_to_world.transform.translation.z])
            rotation = [self.camera_to_world.transform.rotation.x,
                        self.camera_to_world.transform.rotation.y,
                        self.camera_to_world.transform.rotation.z,
                        self.camera_to_world.transform.rotation.w]
            self.get_logger().info(f"Sending transform")
            self.sock.sendall(struct.pack('<L', 1) + struct.pack('<3f', *translation) + struct.pack('<4f', *rotation))
        else:
            self.get_logger().info("Sending empty transform")
            self.sock.sendall(struct.pack('<L', 0))

    def image_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.uint16)

    def send_color_image(self):
        if self.color_image is not None:
            try:
                self.get_logger().info("Sending image")
                _, buffer = cv2.imencode('.jpg', self.color_image)
                size = len(buffer)
                self.sock.sendall(struct.pack('<L', size) + buffer.tobytes())
            except Exception as e:
                self.get_logger().error(f"Failed to send image: {e}")
        else:
            # also send a message of the same format, but not the image
            self.get_logger().info("Sending empty color")
            self.sock.sendall(struct.pack('<L', 0))
    
    def send_depth_image(self):
        if self.depth_image is not None:
            try:
                self.get_logger().info("Sending depth")
                _, buffer = cv2.imencode('.png', self.depth_image)
                size = len(buffer)
                self.sock.sendall(struct.pack('<L', size) + buffer.tobytes())
            except Exception as e:
                self.get_logger().error(f"Failed to send depth: {e}")
        else:
            self.get_logger().info("Sending empty depth")
            self.sock.sendall(struct.pack('<L', 0))
    
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
    socket_sender = SocketSender()
    try:
        socket_sender.wait_for_first_images()
        print("socket connected and first images geT")
        while rclpy.ok():
            rclpy.spin_once(socket_sender, timeout_sec=6.0)
            # socket_sender.listen_tf()
            # socket_sender.wait_handshake("trans")
            # socket_sender.send_transform()
            socket_sender.wait_handshake("color")
            socket_sender.send_color_image()
            socket_sender.wait_handshake("depth")
            socket_sender.send_depth_image()
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        socket_sender.close_socket()
        socket_sender.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()