import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_msgs.msg import GraspResponse
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup


class ArmManipulationNode(Node):
    def __init__(self):
        super().__init__('arm_manipulation_node')
        # Robot arm
        self.robot = InterbotixManipulatorXS(
            robot_model='vx300s',
            group_name='arm',
            gripper_name='gripper',
            gripper_pressure_lower_limit=60,
            gripper_pressure_upper_limit=200
        )
        robot_startup()
        print("Robot initialized")
        self.EXPLORE_VIEW_JOINTS = [0.0, -1.8, 1.36, 0.0, 0.7, 0.0]
        self.CAPTURE_VIEW_JOINTS = [0.0, -0.785, 0.785, 0, 0.785, 0]
        self.robot.arm.core.robot_torque_enable("group", "all", True)
        self.release()
        self.go_to_explore_pose()
        self.robot.gripper.set_pressure(1.0)
        self.create_subscriber()
        print("Node initialized")


    def create_subscriber(self):
        self.prompt_sub = self.create_subscription(String, "/grasp_prompt", self.prompt_callback, 10)

    def prompt_callback(self, msg:String):
        self.get_logger().info(f"Received prompt: {msg.data}")
        if msg.data == "g":
            self.grasp(1)
        else:
            self.release(1)

    def grasp(self, delay=0.0):
        self.get_logger().info("Grasp command issued.")
        self.robot.gripper.grasp(delay=delay)

    def release(self, delay=0.0):
        self.get_logger().info("Release command issued.")
        self.robot.gripper.release(delay=delay)


    def go_to_explore_pose(self):
        self.robot.arm.set_joint_positions(self.EXPLORE_VIEW_JOINTS, moving_time=5.0, blocking=False)

    def go_to_capture_pose(self):
        self.robot.arm.set_joint_positions(self.CAPTURE_VIEW_JOINTS, moving_time=5.0, blocking=False)


def main(args=None):
    rclpy.init(args=args)
    node = ArmManipulationNode()
    rclpy.spin(node)
    node.destroy_node()
    robot_shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()