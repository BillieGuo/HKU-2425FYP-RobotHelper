import rclpy
from rclpy.node import Node
from custom_msgs.srv import SemanticQuery
from rclpy.executors import SingleThreadedExecutor

from .query_point_processor import QueryPointProcessor

class SemanticQueryClient(Node):
    def __init__(self):
        super().__init__('semantic_query_handler')
        self.sem_map_client = self.create_client(
            SemanticQuery, 
            'semantic_query'
        )
        self.query_point_processor = QueryPointProcessor()
        self.point_process_strategy = self.query_point_processor.default_strategy
    
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
    
    def query_service_and_process(self, object_name, threshold=3.1415/6):
        points, similarities, service_names, labels = self.query_service(object_name, threshold)
        if len(points)!=0:
            self.query_point_processor.update_values(points, similarities, service_names, labels)
            return self.point_process_strategy()
        else:
            return None, None, None, None

# 使用示例
def main():
    rclpy.init()
    query_client = SemanticQueryClient()
    executor = SingleThreadedExecutor()
    executor.add_node(query_client)
    try:
        while rclpy.ok():
            object_name = input("Enter object name to query: ")
            similarity = float(input("Enter similarity of query: "))
            points, similarities, service_names, labels = query_client.query_service_and_process(object_name, similarity)
            if points is not None:
                print()
                print(f"Query result for {object_name}:")
                print(f"Point: {points}, Similarity: {similarities}, Service Name: {service_names}, Label: {labels}")
                print()
            else:
                print()
                print(f"No result for {object_name}")
                print()
    except KeyboardInterrupt:
        pass
    finally:
        query_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()