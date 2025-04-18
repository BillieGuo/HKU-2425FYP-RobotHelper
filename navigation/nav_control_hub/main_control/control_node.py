import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Point32
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator
import time
import math
from custom_msgs.srv import SemanticQuery
from sensor_msgs.msg import PointCloud, ChannelFloat32

VISUALIZATION = False

class Navigator(Node):
    def __init__(self):
        super().__init__('Control')
        self.nav2navigator = BasicNavigator()
        self.odom = [0.0, 0.0, 0.0]
        self.target_location = None
        self.prompt = None
        self.explore = None
        
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pose_sub = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        self.navigator2llm_pub = self.create_publisher(String, 'navigator2llm', qos_profile)
        self.llm2navigator_sub = self.create_subscription(String, 'llm2navigator', self.llm_request_callback, qos_profile)
        self.navigator2yolo_pub = self.create_publisher(String, 'navigator2yolo', qos_profile)
        self.yolo2navigator_sub = self.create_subscription(String, 'yolo2navigator', self.yolo_response_callback, qos_profile)
        self.navigator2socket_pub = self.create_publisher(String, 'navigator2socket', qos_profile)
        self.socket2navigator_sub = self.create_subscription(String, 'socket2navigator', self.socket_response_callback, qos_profile)
        # self.timer = self.create_timer(1.0, self.handel_request)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud, 'semantic_points_cloud', 10)

        self.sem_map_client = self.create_client(SemanticQuery, 'semantic_query')
        
        self.get_logger().info('Navigator node initialized.')
    
    def query_service(self, object_name, threshold=3.1415/6):
        req = SemanticQuery.Request()
        req.object_name = object_name
        req.similarity_threshold_rad = threshold
        future = self.sem_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            return future.result().points, future.result().similarities, future.result().corresponding_semantic_service, future.result().labels
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            return None, None, None, None

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
        
    def llm_request_callback(self, msg):
        if msg.data:
            if "[" in msg.data and "]" in msg.data:
                self.prompt = "user location"
                self.target_location = eval(msg.data)
            else:
                self.prompt = msg.data
        
    def yolo_response_callback(self, msg):
        self.explore = True if msg.data == "Success" else False
        self.get_logger().info(f"Exploration result received: {self.explore}")
            
    def requset_location(self, target):
        # OLD: Only query the target location
        # self.get_logger().info(f"Requesting location of {target}")
        # target_locations, _, _, _ = self.query_service(target, 0.2)
        # if not target_locations:
        #     return None
        # return [target_locations[0].x, target_locations[0].y, target_locations[0].z]
        
        # New: Query all the sem points, set the similarity to 2*Pi, get 1st point as location
        self.get_logger().info(f"Requesting all semantic points")
        sem_points, _, _, _ = self.query_service(target, math.pi)
        # self.get_logger().info(f"all sem points:{sem_points}")
        if not sem_points:
            return None
        # Get the first point and viusalize all the points
        self.publish_semantic_points(sem_points)
        return [sem_points[0].x, sem_points[0].y, sem_points[0].z]

    def publish_semantic_points(self, points: list[Point]):
        pointcloud = PointCloud()
        pointcloud.header.frame_id = "map"  # Frame of reference
        pointcloud.header.stamp = self.get_clock().now().to_msg()

        # Convert points to PointCloud format with 32-bit floats
        for point in points:
            geometry_point = Point32()
            geometry_point.x = float(point.x)
            geometry_point.y = float(point.y)
            geometry_point.z = float(point.z)
            pointcloud.points.append(geometry_point)

        # Optionally, add channels (e.g., intensity or other attributes)
        channel = ChannelFloat32()
        channel.name = "intensity"
        channel.values = [1.0] * len(points)  # Example: uniform intensity
        pointcloud.channels.append(channel)

        # Publish the PointCloud
        self.pointcloud_pub.publish(pointcloud)
        self.get_logger().info(f"Published {len(points)} points as PointCloud.")

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
        
    def display_location_point(self, point):
        marker = Marker()
        marker.header.frame_id = "map"  # Frame of reference
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "semantic map object location"
        marker.id = 0
        marker.type = Marker.SPHERE  # Marker type (e.g., SPHERE, CUBE, ARROW, etc.)
        marker.action = Marker.ADD

        # Set the position of the marker
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the scale of the marker
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Set the color of the marker
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue
        marker.color.a = 1.0  # Alpha (transparency)

        # Set the lifetime of the marker (0 means it stays forever)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.marker_pub.publish(marker)
        self.get_logger().info("Published marker")

    # main function
    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if not self.prompt:
                self.prompt = None
                self.explore = None
                continue
            # a prompt comes in with the name of the object to be searched and reached
            # 0. get current pose
            current_pose = self.get_pose()
            self.get_logger().info(f"Current pose: {current_pose}")
            
            if self.prompt == 'request_current_location':
                self.send_navigation_result(self.odom)
                self.prompt = None
                continue
            # 1. pass the object to semantic map to get the location
            # self.target_location = self.odom # hard-coded for now
            if self.prompt != "user location":
                self.target_location = self.requset_location(self.prompt)        
            if self.target_location is None:
                self.get_logger().warning(f'No Target Location, Skip this action.')
                self.prompt = None
                self.explore = False
                self.send_navigation_result(False)
                continue
            self.get_logger().info(f"Location received: {self.target_location}")
            # visualize the retrieved location of object
            self.display_location_point(self.target_location) 
            
            # 2. send the location to the navigator Nav2
            self.send_goal_pos()
            self.get_logger().info(f"Goal sent: {self.target_location}")
            
            while not self.nav2navigator.isTaskComplete():
                feedback = self.nav2navigator.getFeedback()
                if (self.odom[0] - self.target_location[0])**2 + (self.odom[1] - self.target_location[1])**2 < 0.3**2:
                    self.nav2navigator.cancelTask()
                    self.get_logger().info(f"Goal reached, cancelling task.")
                    break
                self.get_logger().info(f'{feedback.navigation_time}')
                if int(feedback.navigation_time.sec) > 60:
                    self.nav2navigator.cancelTask()
                    self.get_logger().info(f"Timeout, cancelling task.")
                    self.prompt = None
                    self.explore = None
                    self.send_navigation_result(False)
                    break
            if self.prompt is None:
                continue
            self.get_logger().info(f"Location Reached!")
            if self.prompt == "user location":
                self.send_navigation_result(True)   
                self.clean_up()
                continue
            # 3. after reaching the location, conduct exploration
            self.request_exploration(self.prompt)
            self.get_logger().info(f"Exploration started.")
            exploration_start_time = time.time()
            exploration_timeout = 60  # Timeout in seconds
            while self.explore is None:
                rclpy.spin_once(self)
                if time.time() - exploration_start_time > exploration_timeout:
                    self.get_logger().warning(f"Exploration timed out after {exploration_timeout} seconds.")
                    self.prompt = None
                    self.explore = False
                    self.request_exploration("stop")
                    break
                # self.get_logger().info(f"Exploration in progress.")
                continue
            self.get_logger().info(f"Exploration done.")
            # 4. send the exploration result to the LLM
            result = False if self.prompt is None else True
            self.send_navigation_result(result)            
            
            self.clean_up()
    
    def send_goal_pos(self):
        try:
            x, y, _ = self.target_location
            theta = self.odom[2]
            # x, y, theta = [0.0, 0.0, 0.0]
        except ValueError:
            x, y, theta = self.odom
            self.get_logger().info(f'Invalid point, stay at current position')
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

        # direct post
        # self.goal_publisher.publish(goal) 
        # via API
        self.nav2navigator.goToPose(goal) #, behavior_tree='Pause Near Goal-Obstacle')
        self.get_logger().info(f'Goal pose sent: {goal}')

    def get_pose(self):
        return self.odom

    def clean_up(self):
        self.prompt = None
        self.explore = None
        self.request_exploration("stop")
        self.nav2navigator.cancelTask()
        self.get_logger().info(f"Reset.")
        
def main():
    rclpy.init()
    node = Navigator()
    node.run()
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()