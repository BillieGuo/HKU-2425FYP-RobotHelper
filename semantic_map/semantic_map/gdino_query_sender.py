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

class GDinoQuerySender(Node):
    def __init__(self):
        super().__init__('image_transform_listener')

        config_file = read_config("config_gdino_query_sender")
        self.port_num = config_file['socket_connection']['port_num']

        # Subscribers for color and depth images
        self.color_image_sub = self.create_subscription(
            Image,
            '/grasp_module/D435i/color/image_raw',
            self.color_image_callback,
            10)

        # Data storage
        self.bridge = CvBridge()
        self.color_image_msg = None
        self.depth_image_msg = None
        self.transform_msg = None

        # Socket server setup
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(("0.0.0.0", self.port_num))
        self.server_socket.listen(1)
        self.client_socket = None
        self.get_logger().info("Waiting for client connection...")
        self.client_socket, _ = self.server_socket.accept()
        self.get_logger().info("Client connected.")

        self.timer = self.create_timer(0.1, self.send_query)

    def color_image_callback(self, msg):
        self.color_image_msg = msg

    def send_query(self):
        if self.color_image_msg is None:
            return
        query = input("Enter query: ")
        # Prepare data
        data = {
            "color_image": self.color_image_msg if self.color_image_msg else None,
            "query": query
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
            
            self.receive_data()

        if not self.client_socket:
            # end entire program
            self.get_logger().info("No client connected. Exiting.")
            self.destroy_node()
            rclpy.shutdown()
            exit(0)
    
    def receive_data(self):
        # The client has send back data similarly to server, however, the field are xy, phrases, and logits

        try:
            # Receive size of the byte stream
            size_data = self.client_socket.recv(4)
            if not size_data:
                self.get_logger().warning("Server disconnected.")
                self.client_socket.close()
                return
            size = int.from_bytes(size_data, 'big')
            # Receive the byte stream
            byte_stream = b""
            while len(byte_stream) < size:
                packet = self.client_socket.recv(size - len(byte_stream))
                if not packet:
                    self.get_logger().warning("Server disconnected.")
                    self.client_socket.close()
                    return
                byte_stream += packet
            # Deserialize the data
            data = pickle.loads(byte_stream)
            # Separate images and transform
            xy = data.get("xy")
            phrases = data.get("phrases")
            logits = data.get("logits")
            # Log the separated data
            self.get_logger().info(f"Received XY: {xy}")
            self.get_logger().info(f"Received Phrases: {phrases}")
            self.get_logger().info(f"Received Logits: {logits}")
        except Exception as e:
            self.get_logger().error(f"Error receiving data: {e}")
            self.client_socket.close()

    def destroy_node(self):
        # Close sockets on shutdown
        if self.client_socket:
            self.client_socket.close()
        self.server_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GDinoQuerySender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
