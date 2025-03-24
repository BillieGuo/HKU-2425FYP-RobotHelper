import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(1.0, self.send_goal)

    def send_goal(self):
        prompt = input(f'Goal point (in [x,y]): ') # [x, y]
        if not prompt:
            return
        try:
            x, y = map(float, prompt.strip('[]').split(','))
        except:
            self.get_logger().info(f'Wrong format! Please use [x,y] instead.')
            return
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.publisher.publish(goal)
        self.get_logger().info('Goal pose sent!')


def main():
    rclpy.init()
    node = GoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()