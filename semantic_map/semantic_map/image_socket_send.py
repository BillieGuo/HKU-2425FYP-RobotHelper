import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import struct
import time

import rclpy
from geometry_msgs.msg import TransformStamped
import tf_transformations
import numpy as np

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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
        self.updating_color_image = None
        self.updating_depth_image = None
        self.socket_setup()

        self.camera_frame = camera_frame
        self.world_frame = world_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_to_world = None
    
    def wait_for_first_images(self):
        while self.updating_color_image is None or self.updating_depth_image is None:
            rclpy.spin_once(self)
            time.sleep(0.1)

    def socket_setup(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.connect_to, 8812))

    def listen_tf(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.camera_frame
        to_frame_rel = self.world_frame

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return False
        
        self.get_logger().info(
            f'{to_frame_rel} to {from_frame_rel}: \n'
            f'x: {t.transform.translation.x}\n'
            f'y: {t.transform.translation.y}\n'
        )
        self.camera_to_world = t
        return True

    def send_transform(self):
        if self.camera_to_world is not None:
            translation = [self.camera_to_world.transform.translation.x,
                                    self.camera_to_world.transform.translation.y,
                                    self.camera_to_world.transform.translation.z]
            rotation = [self.camera_to_world.transform.rotation.x,
                        self.camera_to_world.transform.rotation.y,
                        self.camera_to_world.transform.rotation.z,
                        self.camera_to_world.transform.rotation.w]
            self.get_logger().info(f"Sending transform")
            self.sock.sendall(struct.pack('<L', 1) + struct.pack('<3f', *translation) + struct.pack('<4f', *rotation))
        else:
            self.get_logger().info("Sending empty transform")
            translation = [0,
                        0,
                        0]
            rotation = [0,
                        0,
                        0,
                        0]
            self.sock.sendall(struct.pack('<L', 1) + struct.pack('<3f', *translation) + struct.pack('<4f', *rotation))

    def image_callback(self, msg):
        self.updating_color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        self.updating_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.uint16)
    
    def image_fix(self):
        self.color_image = self.updating_color_image
    
    def depth_fix(self):
        self.depth_image = self.updating_depth_image

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
    socket_sender = SocketSender(world_frame="map", camera_frame="D435i_color_optical_frame", connect_to='fyp')
    try:
        socket_sender.wait_for_first_images()
        print("socket connected and first images geT")
        while rclpy.ok():
            tf_get = False
            while rclpy.ok() and tf_get == False:
                rclpy.logging.get_logger("tf_get").info("Waiting for tf")
                socket_sender.depth_fix()
                socket_sender.image_fix()
                rclpy.spin_once(socket_sender, timeout_sec=5.0)
                tf_get = socket_sender.listen_tf()
                
            socket_sender.wait_handshake("depth")
            socket_sender.send_depth_image()
            socket_sender.wait_handshake("color")
            socket_sender.send_color_image()
            socket_sender.wait_handshake("trans")
            socket_sender.send_transform()

    except KeyboardInterrupt:
        pass
    finally:
        socket_sender.close_socket()
        socket_sender.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()