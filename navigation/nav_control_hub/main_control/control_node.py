import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
from tf_transformations import quaternion_from_euler


class Navigator(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.odom = [0.0, 0.0, 0.0]
        self.prompt = None
        self.explore = None
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/current_pose', self.odom_callback, 10)
        self.llm2navigator_sub = self.create_subscription(String, 'llm2navigator', self.llm_request_callback, 10)
        self.yolo2llm_sub = self.create_subscription(String, 'yolo2llm', self.yolo_request_callback, 10)
        self.timer = self.create_timer(1.0, self.handel_request)

    def odom_callback(self, msg):
        self.get_logger().info(f'Current pose: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}')
        self.get_logger().info(f'Current orientation: {msg.pose.orientation.w:.2f}')
        self.odom = [msg.pose.position.x, msg.pose.position.y, msg.pose.orientation.w]
        
    def llm_request_callback(self, msg):
        if msg.data:
            self.prompt = msg.data
        
    def yolo_request_callback(self, msg):
        if msg.data:
            self.explore = True
            
        
    def handel_request(self):
        if not self.prompt:
            return
        
        
        self.prompt = None
        self.explore = False
        pass
    
    def send_goal_pos(self):
        try:
            x, y, theta = map(float, self.prompt.strip('[]').split(','))
        except ValueError:
            self.get_logger().info(f'Wrong format! Please use [x,y,theta] instead.')
            return
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y

        # Convert theta to quaternion
        q = quaternion_from_euler(0, 0, theta)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        self.goal_publisher.publish(goal)
        self.get_logger().info(f'Goal pose sent: x={x}, y={y}, theta={theta}')

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