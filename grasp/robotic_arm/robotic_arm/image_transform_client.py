import rclpy
from rclpy.node import Node
import socket
import pickle

class ImageTransformClient(Node):
    def __init__(self):
        super().__init__('image_transform_client')

        # Socket client setup
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(("0.0.0.0", 34567))
        self.get_logger().info("Connected to server.")

        # Timer to periodically receive and process data
        self.timer = self.create_timer(2.0, self.receive_data)

    def receive_data(self):
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
            color_image_msg = data.get("color_image")
            depth_image_msg = data.get("depth_image")
            transform = data.get("transform")

            # Log the separated data
            self.get_logger().info(f"Received Color Image: {color_image_msg}")
            self.get_logger().info(f"Received Depth Image: {depth_image_msg}")
            self.get_logger().info(f"Received Transform: {transform}")
        except Exception as e:
            self.get_logger().error(f"Error receiving data: {e}")
            self.client_socket.close()

    def destroy_node(self):
        # Close socket on shutdown
        self.client_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageTransformClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
