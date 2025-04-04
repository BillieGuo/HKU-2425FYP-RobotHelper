import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy

class Navigator(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.odom = [0.0, 0.0, 0.0]
        self.target_location = None
        self.prompt = None
        self.explore = None
        
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # self.pose_sub = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        self.navigator2llm_pub = self.create_publisher(String, 'navigator2llm', qos_profile)
        self.llm2navigator_sub = self.create_subscription(String, 'llm2navigator', self.llm_request_callback, qos_profile)
        self.navigator2yolo_pub = self.create_publisher(String, 'navigator2yolo', qos_profile)
        self.yolo2navigator_sub = self.create_subscription(String, 'yolo2navigator', self.yolo_response_callback, qos_profile)
        self.navigator2socket_pub = self.create_publisher(String, 'navigator2socket', qos_profile)
        self.socket2navigator_sub = self.create_subscription(String, 'socket2navigator', self.socket_response_callback, qos_profile)
        # self.timer = self.create_timer(1.0, self.handel_request)

        self.get_logger().info('Navigator node initialized.')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        # Convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])
        self.odom = [x, y, yaw] # x, y and theta
        # self.get_logger().info(f'Current odom: {self.odom}')
        
    def llm_request_callback(self, msg):
        if msg.data:
            self.prompt = msg.data
        
    def yolo_response_callback(self, msg):
        self.explore = True if msg.data == "Success" else False
        self.get_logger().info(f"Exploration result received: {self.explore}")
            
    def requset_location(self, target):
        self.get_logger().info(f"Requesting location of {target}")
        msg = String()
        msg.data = target
        self.navigator2socket_pub.publish(msg)    
        
    def request_exploration(self, target):
        self.get_logger().info(f"Requesting exploration of {target}")
        msg = String()
        msg.data = target
        self.navigator2yolo_pub.publish(msg)
        
    def send_navigation_result(self, result):
        self.get_logger().info(f"Sending navigation result to LLM: {result}")
        msg = String()
        msg.data = str(result)
        self.navigator2llm_pub.publish(msg)
        
    def socket_response_callback(self, msg):
        if msg.data:
            self.get_logger().info(f"Received from socket: {msg.data}")
            self.target_location = map(float, msg.data.strip('[]').split(','))
        
    # main function
    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if not self.prompt:
                return
            # a prompt comes in with the name of the object to be searched and reached
            # 0. get current pose
            current_pose = self.get_pose()
            self.get_logger().info(f"Current pose: {current_pose}")
            # 1. pass the object to semantic map to get the location
            self.requset_location(self.prompt)        
            # wait for the location to be received
            # while not self.target_location:
            #     continue
            self.get_logger().info(f"Location received: {self.target_location}")
            # 2. send the location to the navigator Nav2
            self.send_goal_pos()
            # self.target_location = None
            self.get_logger().info(f"Goal sent: {self.target_location}")
            # 3. after reaching the location, conduct exploration
            self.request_exploration(self.prompt)
            self.get_logger().info(f"Exploration started.")
            while not self.explore:
                rclpy.spin_once(self)
                self.get_logger().info(f"Exploration in progress.")
                continue
            # 4. send the exploration result to the LLM
            self.send_navigation_result(True)
            
            self.prompt = None
            self.explore = False
    
    def send_goal_pos(self):
        try:
            # x, y, theta = self.target_location
            x, y, theta = [0.0, 0.0, 0.0]
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
    node.run()
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()