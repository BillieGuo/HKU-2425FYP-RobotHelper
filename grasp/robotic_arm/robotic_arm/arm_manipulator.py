import rclpy
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from enum import Enum

class ArmState(Enum):
    CAPTURING = 1
    GRASPING = 2

class ArmManipulator(InterbotixManipulatorXS):
    def __init__(self):
        super().__init__(
            robot_model='vx300s',
            group_name='arm',
            gripper_name='gripper',
        )
        self.state = ArmState.CAPTURING

    def run(self):
        try:
            robot_startup()
            while rclpy.ok():
                self.handle_state()
                
        except KeyboardInterrupt:
            robot_shutdown()

    def handle_state(self):
        if self.state == ArmState.CAPTURING:
            self.capture()
        elif self.state == ArmState.GRASPING:
            self.grasp()

    def capture(self):
        # Capturing logic
        # Wait for a specific command to transition to the next state
        pass

    def grasp(self):
        # Grasping logic
        # Move the arm and the gripper to grasp an object
        pass

def main(args=None):
    rclpy.init(args=args)
    robotic_arm = ArmManipulator()
    robotic_arm.run()

if __name__ == "__main__":
    main()