import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformException
from cv_bridge import CvBridge
import time
import socket
import pickle
from .utils import read_config

class ImageTransformListener(Node):
    def __init__(self, camera_frame="D435i_color_optical_frame", world_frame="map"):
        super().__init__('image_transform_listener')

        config_file = read_config("config_image_transform_listener")
        self.port_num = config_file['socket_connection']['port_num']
        self.update_interval = config_file['socket_connection']['update_interval']

        # Subscribers for color and depth images
        self.color_image_sub = self.create_subscription(
            Image,
            '/grasp_module/D435i/color/image_raw',
            self.color_image_callback,
            10)
        self.depth_image_sub = self.create_subscription(
            Image,
            '/grasp_module/D435i/aligned_depth_to_color/image_raw',
            self.depth_image_callback,
            10)

        # Transform listener
        self.camera_frame = camera_frame
        self.world_frame = world_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Data storage
        self.bridge = CvBridge()
        self.color_image_msg = None
        self.depth_image_msg = None
        self.transform_msg = None

        # Timer to periodically collect and print data
        self.timer = self.create_timer(self.update_interval, self.collect_and_print_data)

        # Socket server setup
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(("0.0.0.0", self.port_num))
        self.server_socket.listen(1)
        self.client_socket = None
        self.get_logger().info("Waiting for client connection...")
        self.client_socket, _ = self.server_socket.accept()
        self.get_logger().info("Client connected.")

    def color_image_callback(self, msg):
        self.color_image_msg = msg

    def depth_image_callback(self, msg):
        self.depth_image_msg = msg

    def listen_transform(self):
        try:
            self.transform_msg = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.camera_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warning(f"Could not transform {self.world_frame} to {self.camera_frame}: {ex}")
            self.transform_msg = None

    def collect_and_print_data(self):
        # Update the transform
        self.listen_transform()
        if self.transform_msg is None:
            return

        # Prepare data
        data = {
            "color_image": self.color_image_msg if self.color_image_msg else None,
            "depth_image": self.depth_image_msg if self.depth_image_msg else None,
            "transform": self.transform_msg
        }

        # Serialize and send data if client is connected
        if self.client_socket:
            try:
                byte_stream = pickle.dumps(data)
                self.client_socket.sendall(len(byte_stream).to_bytes(4, 'big'))
                self.client_socket.sendall(byte_stream)
                self.get_logger().info("Data sent to client.")
            except Exception as e:
                self.get_logger().error(f"Error sending data to client: {e}")
                self.client_socket.close()
                self.client_socket = None
        if not self.client_socket:
            # end entire program
            self.get_logger().info("No client connected. Exiting.")
            self.destroy_node()
            rclpy.shutdown()
            exit(0)

        # Print raw messages
        self.get_logger().info("Collected Data:")
        # if self.color_image_msg:
        #     self.get_logger().info(f"Color Image: {self.color_image_msg}")
        # else:
        #     self.get_logger().info("Color Image: None")

        # if self.depth_image_msg:
        #     self.get_logger().info(f"Depth Image: {self.depth_image_msg}")
        # else:
        #     self.get_logger().info("Depth Image: None")

        if self.transform_msg:
            self.get_logger().info(f"Transform: {self.transform_msg}")
        else:
            self.get_logger().info("Transform: None")

    def destroy_node(self):
        # Close sockets on shutdown
        if self.client_socket:
            self.client_socket.close()
        self.server_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageTransformListener(camera_frame="D435i_color_optical_frame", world_frame="map")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
