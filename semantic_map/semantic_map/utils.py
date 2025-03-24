# class SocketReceiver
import socket
import struct
import numpy as np

# class DepthCamSocketMaintainer
import rclpy
import cv2
from PIL import Image as PILImage

# class RealSensePointCalculator
from cv_bridge import CvBridge
import pyrealsense2 as rs


class SocketReceiver:
    '''
    A class to handle socket communication for receiving transformation, color, depth, and info data.

    Attributes:
        server_socket (socket.socket): The server socket object.
        conn (socket.socket): The connection socket object.
        addr (tuple): The address bound to the socket.

    Methods:
        socket_connect(port_num=5001): Establishes a socket connection on the given port.
        send_handshake(handshake_message): Sends a handshake message to the connected client.
        receive_data(variable_length=False, formats=["<3f", "<4f"]): Receives data from the socket.
    '''
    def __init__(self):
        self.server_socket = None
        self.conn, self.addr = None, None

    def socket_connect(self, port_num=5001):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(("0.0.0.0", port_num))
        self.server_socket.listen(1)
        self.conn, self.addr = self.server_socket.accept()

    def send_handshake(self, handshake_message):
        # print(f"Sending handshake message:{handshake_message}")
        self.conn.sendall(handshake_message.encode())

    def receive_data(self, variable_length=False, formats="<3f4f"):
        '''
        Receives data from the socket, either of variable length or fixed length based on provided formats.
        if the data is fixed length, the sender needs to send a valid data flag 1 of "<L" before sending the data.
        if the data is variable length, the sender needs to send the data size of "<L" before sending the data.

        Args:
            variable_length (bool): If True, receives data of variable length. If False, receives fixed length data.
            formats (string): A struct format string to unpack the data.

        Returns:
            data (bytes or list): The received data. 
                                  If variable_length is True, returns the raw data bytes.
                                  If variable_length is False, returns a list of unpacked numpy arrays.
                                  e.g. [np.array, np.array, ...]
        '''
        if variable_length:
            data_size = struct.unpack("<L", self.conn.recv(4))[0]
            data = b""
            if data_size == 0:
                print("Received empty data")
                return None
            else:
                while len(data) < data_size:
                    packet = self.conn.recv(4096)
                    if not packet:
                        break
                    data += packet
                if len(data) == data_size:
                    return data
                else:
                    raise Exception("Data size does not match")
        else:
            if len(formats) == 0:
                print("No format provided")
                return None
            data_valid = struct.unpack("<L", self.conn.recv(4))[0]
            if data_valid == 1:
                data = self.conn.recv(struct.calcsize(formats))
                if len(data) == struct.calcsize(formats):
                    return np.array(struct.unpack(formats, data))
                else:
                    print("Received invalid data")
                    return None
            else:
                print("Received invalid data")
                return None

class DepthCamSocketMaintainer(SocketReceiver):
    def __init__(self):
        super().__init__()
        self.info = None
        self.trans = None
        self.pil_image = None
        self.depth = None
    
    def receive_info(self):
        self.info = self.receive_data(variable_length=False, formats="<2I4d")
        # print(f"Received info: {self.info}")
    
    def return_info(self):
        return self.info
    
    def receive_trans(self):
        # first 3 floats are translation, next 4 floats are rotation
        self.trans = self.receive_data(variable_length=False, formats="<3f4f")
        # print(f"Received trans: {self.trans[:3]}, rot: {self.trans[3:]}")
    
    def return_trans(self):
        return self.trans
    
    def receive_color(self):
        data = self.receive_data(variable_length=True)
        if data is not None:
            np_array = np.frombuffer(data, np.uint8)
            color_img = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
            color = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
            self.pil_image = PILImage.fromarray(color)
            # print(f"Received color: {color.shape}")
        else:
            print("Received empty color data")
            return None
    
    def return_color(self):
        return self.pil_image
    
    def receive_depth(self):
        data = self.receive_data(variable_length=True)
        if data is not None:
            np_array = np.frombuffer(data, np.uint8)
            self.depth = cv2.imdecode(np_array, cv2.IMREAD_UNCHANGED)
            # print(f"Received depth: {self.depth.shape}")
        else:
            print("Received empty depth data")
            return None
    
    def return_depth(self):
        return self.depth
    
    def handshake_receive_data(self):
        self.send_handshake("info")
        self.receive_info()
        self.send_handshake("trans")
        self.receive_trans()
        self.send_handshake("color")
        self.receive_color()
        self.send_handshake("depth")
        self.receive_depth()

    def init_socket(self, port_num=5555):
        self.socket_connect(port_num)

        while (
            rclpy.ok() and 
            (self.pil_image is None
            or self.depth is None
            or self.trans is None
            or self.info is None)):
            self.handshake_receive_data()

class RealSensePointCalculator:
    def __init__(self, depth_frame_size=[480, 640], image_frame_size=[480, 480]):
        self.bridge = CvBridge()
        self.depth_image = None
        self.intrinsics = rs.intrinsics()
        self.intrinsics.width = 848
        self.intrinsics.height = 480
        self.intrinsics.ppx = 430.2650451660156
        self.intrinsics.ppy = 238.0896759033203
        self.intrinsics.fx = 425.21417236328125
        self.intrinsics.fy = 425.21417236328125
        self.intrinsics.model = rs.distortion.none
        self.intrinsics.coeffs = [0.0 for i in range(5)]
        self.depth_frame_size = depth_frame_size
        self.x_offset = self.depth_frame_size[1] // 2 - image_frame_size[1] // 2
        self.y_offset = self.depth_frame_size[0] // 2 - image_frame_size[0] // 2

    def update_depth(self, depth_img):
        self.depth_image = depth_img

    def update_intr(self, camera_info):
        self.intrinsics.width = int(camera_info[0])
        self.intrinsics.height = int(camera_info[1])
        self.intrinsics.ppx = camera_info[2]
        self.intrinsics.ppy = camera_info[3]
        self.intrinsics.fx = camera_info[4]
        self.intrinsics.fy = camera_info[5]

    def calculate_point(self, pixel_y, pixel_x):
        # Deprojection of depth camera pixel to 3D point
        depth_pixel_y = pixel_y 
        depth_pixel_x = pixel_x 
        depth = (
            self.depth_image[depth_pixel_y, depth_pixel_x] * 0.001
        )  # Convert from mm to meters
        point = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [pixel_x, pixel_y], depth
        )
        point = [point[2], -point[0], -point[1]]
        return point

    def calculate_point_with_offset(self, pixel_y, pixel_x):
        # Deprojection of depth camera pixel to 3D point
        depth_pixel_y = pixel_y + self.y_offset
        depth_pixel_x = pixel_x + self.x_offset
        depth = (
            self.depth_image[depth_pixel_y, depth_pixel_x] * 0.001
        )  # Convert from mm to meters
        point = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [depth_pixel_x, depth_pixel_y], depth
        )
        point = [point[2], -point[0], -point[1]]
        return point

