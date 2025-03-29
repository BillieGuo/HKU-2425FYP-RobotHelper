import math
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import time

class FrameListener(Node):

    def __init__(self):
        super().__init__('tf2_frame_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_frame = "map"
        self.source_frame = "D435i_color_optical_frame"

        # self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = self.source_frame

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        self.get_logger().info(
            f'{to_frame_rel} to {from_frame_rel}: \n'
            f'x: {t.transform.translation.x}\n'
            f'y: {t.transform.translation.y}\n'
        )


def main():
    rclpy.init()
    node = FrameListener()
    try:
        # rclpy.spin(node)
        while rclpy.ok():
            rclpy.spin_once(node)  # Spin once to process callbacks and check for a KeyboardInterrupt
            node.on_timer()  # Call the on_timer method to perform the transformation and log the result
            # You can add a delay here if needed to control the loop rate
            # time.sleep(0.5)  # Adjust the delay as needed
            
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()