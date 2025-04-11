import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener


class RobotArmControlNode(Node):
    def __init__(self):
        super().__init__('robot_arm_control_node')

        # Initialize the robot manipulator
        self.robot = InterbotixManipulatorXS(robot_model='wx200')

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber to listen for commands
        self.subscription = self.create_subscription(
            String,
            'robot_arm_command',
            self.command_callback,
            10
        )

        # Publisher to notify when actions are complete
        self.publisher = self.create_publisher(String, 'robot_arm_status', 10)

        # Timer to delay the transform processing until the listener is ready
        self.create_timer(1.0, self.delayed_transform_processing)

        # Starting process
        self.starting_process()

    def starting_process(self):
        self.get_logger().info('Starting process: Moving arm and controlling gripper...')
        
        # Close the gripper
        self.robot.gripper.close(blocking=True)
        self.get_logger().info('Gripper closed.')

        # Move the arm to a specific position
        self.robot.arm.set_joint_positions([0.0, -1.0, 1.0, 0.5], blocking=True)
        self.get_logger().info('Arm moved to the starting position.')

    def delayed_transform_processing(self):
        """
        Delay the transform processing to ensure the listener is ready.
        """
        self.get_logger().info('Processing and publishing transform after initialization...')
        self.process_and_publish_transform('base_link', 'ee_gripper_link', 'processed_frame')

    def process_and_publish_transform(self, target_frame: str, source_frame: str, new_frame: str):
        """
        Look up a static transform, process it, and publish another static transform.

        :param target_frame: The target frame to look up the transform to.
        :param source_frame: The source frame to look up the transform from.
        :param new_frame: The name of the new frame to publish the processed transform.
        """
        try:
            # Lookup the transform
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            self.get_logger().info(f"Transform from {source_frame} to {target_frame} found.")

            # Process the transform (example: apply a translation offset)
            processed_transform = TransformStamped()
            processed_transform.header.stamp = self.get_clock().now().to_msg()
            processed_transform.header.frame_id = target_frame
            processed_transform.child_frame_id = new_frame
            processed_transform.transform.translation.x = transform.transform.translation.x + 0.1
            processed_transform.transform.translation.y = transform.transform.translation.y
            processed_transform.transform.translation.z = transform.transform.translation.z
            processed_transform.transform.rotation = transform.transform.rotation

            # Publish the new transform
            self.tf_broadcaster.sendTransform(processed_transform)
            self.get_logger().info(f"Published new transform for frame {new_frame}.")
        except Exception as e:
            self.get_logger().error(f"Failed to lookup or process transform: {e}")

    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')

        # Release the gripper
        self.robot.gripper.open(blocking=True)
        self.get_logger().info('Gripper opened.')

        # Move the arm to a specific pose
        self.robot.arm.set_joint_positions([0.5, -0.5, 0.5, 0.0], blocking=True)
        self.get_logger().info('Arm moved to the target pose.')

        # Close the gripper
        self.robot.gripper.close(blocking=True)
        self.get_logger().info('Gripper closed.')

        # Publish a status message
        status_msg = String()
        status_msg.data = 'Action completed'
        self.publisher.publish(status_msg)
        self.get_logger().info('Published status: Action completed.')


def main(args=None):
    rclpy.init(args=args)
    node = RobotArmControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        node.robot.core.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
