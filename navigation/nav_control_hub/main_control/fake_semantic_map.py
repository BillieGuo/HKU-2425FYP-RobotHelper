import rclpy
from rclpy.node import Node
from custom_msgs.srv import SemanticQuery
from geometry_msgs.msg import Point

class FakeSemanticMap(Node):
    def __init__(self):
        super().__init__('fake_semantic_map')
        self.srv = self.create_service(SemanticQuery, 'semantic_query', self.semantic_query_callback)

    def semantic_query_callback(self, request, response):
        self.get_logger().info(f'Received request for object: {request.object_name} with threshold: {request.similarity_threshold_rad}')
        
        # Allow user to input only point coordinates
        self.get_logger().info("Please provide the response data for the service.")
        
        # Collect points
        response.points = []
        self.get_logger().info("Enter points (comma-separated, e.g., 1.0,2.0,3.0):")
        points_input = input()
        coordinates = [float(x) for x in points_input.split(',')]
        if len(coordinates) % 3 != 0:
            self.get_logger().error("Invalid input: Points must be in sets of three (x, y, z).")
            return response
        response.points = [Point(x=coordinates[i], y=coordinates[i+1], z=coordinates[i+2]) for i in range(0, len(coordinates), 3)]

        return response

def main(args=None):
    rclpy.init(args=args)
    node = FakeSemanticMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()