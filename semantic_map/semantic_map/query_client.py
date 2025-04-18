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
        self.point_process_strategy = self.query_point_processor.max_of_sim_1_sort_confs
    
    def query_service(self, object_name, threshold=3.1415/6):
        req = SemanticQuery.Request()
        req.object_name = object_name
        req.similarity_threshold_rad = threshold
        future = self.sem_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            return future.result().points, future.result().similarities, future.result().corresponding_semantic_service, future.result().labels, future.result().confs
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            return None, None, None, None
    
    def query_service_and_process(self, object_name, threshold=3.1415/6):
        points, similarities, service_names, labels, confs = self.query_service(object_name, threshold)
        if points == []:
            return None, None, None, None, None
        else:
            return points[0], similarities[0], service_names[0], labels[0], confs[0]


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
            points, similarities, service_names, labels, conf = query_client.query_service_and_process(object_name, similarity)
            if points is not None:
                print()
                print(f"Query result for {object_name}:")
                print(f"Point: {points}, Similarity: {similarities}, Service Name: {service_names}, Label: {labels}, Conf: {conf}")
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