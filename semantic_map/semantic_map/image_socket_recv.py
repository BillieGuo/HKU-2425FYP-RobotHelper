from .utils import DepthCamSocketMaintainer
import rclpy
from rclpy.node import Node
import traceback
from sensor_msgs.msg import Image

class SocketPublisher(Node):
    def __init__(self):
        super().__init__('socket_publisher')
        self.color_pub = self.create_publisher(Image, 'color', 10)
        self.depth_pub = self.create_publisher(Image, 'depth', 10)
    
    def publish_images(self, color_image, depth_image):
        color_msg = Image()
        color_msg.data = color_image.tobytes()
        color_msg.width = 640
        color_msg.height = 480
        color_msg.encoding = 'rgb8'
        color_msg.step = 1920
        color_msg.header.frame_id = 'color'
        color_msg.header.stamp = self.get_clock().now().to_msg()
        self.color_pub.publish(color_msg)
        
        depth_msg = Image()
        depth_msg.data = depth_image.tobytes()
        depth_msg.width = 640
        depth_msg.height = 480
        depth_msg.encoding = '16UC1'
        depth_msg.step = 1280
        depth_msg.header.frame_id = 'depth'
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_pub.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    dcm = DepthCamSocketMaintainer()
    dcm.socket_connect(port_num=8812)
    sp = SocketPublisher()
    print("Socket connected")
    try:
        while rclpy.ok():
            dcm.send_handshake("color")
            dcm.receive_color()
            color_image = dcm.return_color()
            dcm.send_handshake("depth")
            dcm.receive_depth()
            depth_image = dcm.return_depth()
            sp.publish_images(color_image, depth_image)

    except KeyboardInterrupt:
        print("Shutting down due to KeyboardInterrupt")
    except Exception as e:
        print(f"An error occurred: {e}")
        traceback.print_exc()
    finally:
        dcm.socket_close()
        print("Socket closed")

    rclpy.shutdown()
