import serial
import struct
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.timer = self.create_timer(0.0001, self.read_from_serial)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.last_time = 0
        self.variables = (0, 0, 0) # x, y, theta
        self.tf_broadcaster1 = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cmdvel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmdvel_callback, 10)

    def read_from_serial(self):
        if self.ser.in_waiting > 0:
            data = self.ser.read(1)  # Read one byte
            uint8_value = struct.unpack('B', data)[0]
            if uint8_value == 0xA5:
                # Read 13 bytes (3 variables of 4 bytes each + 1 byte for EOF)
                data = self.ser.read(13)
                if len(data) == 13:
                    variables = struct.unpack('fff', data[:12])
                    
                    # Get the last byte as EOF
                    eof_value = struct.unpack('B', data[12:])[0]
                        
                    if eof_value == 0x5A:
                        self.variables = variables # Update the variables
                        self.get_logger().info(f"Received variables: {variables}")
                        # self.get_logger().info('Broadcasted transform from odom to base_link')
                        # time_diff = time.time() - self.last_time
                        # self.last_time = time.time()
                        # self.get_logger().info(f"Published variables: {variables}, Time diff: {time_diff}")
                                        
        
    def cmdvel_callback(self, msg):
        self.get_logger().info(f"Received cmd_vel: {msg}")
        # Convert the Twist message to 3 variables
        x, y, theta = 0.0, 0.0, 0.0
        x = msg.linear.x
        y = msg.linear.y
        theta = msg.angular.z
        self.get_logger().info(f"Converted cmd_vel: {x}, {y}, {theta}")
        # Pack the 3 variables into a byte array
        tx_buf = bytearray()
        tx_buf.append(0x5A)
        # tx_buf.extend(struct.pack('f', float(self.variables[0]*1000)))
        # tx_buf.extend(struct.pack('f', float(self.variables[1]*1000)))
        # tx_buf.extend(struct.pack('f', float(self.variables[2]*1000)))
        tx_buf.extend(struct.pack('<fff', x, y, theta))
        tx_buf.append(0xA5)
        # Send the byte array to the serial port
        self.ser.write(tx_buf)
        self.get_logger().info(f"Sent cmd_vel: {tx_buf}")
        pass

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.ser.close()
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()