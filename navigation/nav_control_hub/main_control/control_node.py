import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class Navigator(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.odom = [0.0, 0.0, 0.0]
        self.target = None
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/current_pose', self.odom_callback, 10)
        self.llm_sub = self.create_subscription(String, 'llm2navigator', self.llm_request_callback, 10)
        self.timer = self.create_timer(1.0, self.send_goal)

    def odom_callback(self, msg):
        self.get_logger().info(f'Current pose: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}')
        self.get_logger().info(f'Current orientation: {msg.pose.orientation.w:.2f}')
        self.odom = [msg.pose.position.x, msg.pose.position.y, msg.pose.orientation.w]
        
    def llm_request_callback(self, msg):
        if msg.data:
            self.target = msg.data
        
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
        self.goal_publisher.publish(goal)
        self.get_logger().info('Goal pose sent!')

    def get_pose(self):
        return self.odom

def main():
    rclpy.init()
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()