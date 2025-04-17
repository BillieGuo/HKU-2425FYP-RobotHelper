import socket
import rclpy
import rclpy.serialization
from rclpy.node import Node
import struct
import time

import rclpy
import numpy as np
from rclpy.executors import MultiThreadedExecutor

from custom_msgs.msg import ObjectSem
from custom_msgs.srv import SemanticQuery

from .query_point_processor import *
from .utils import read_config

class SocketSender(Node):
    def __init__(self, connect_to='fyp'):
        super().__init__('image_subscriber_socket_sender')

        config_file = read_config("config_query_semantic_map")
        self.port_num = config_file['socket_connection']['port_num']
        self.connect_to = connect_to
        self.socket_setup()
        self.query_service = self.create_service(
            SemanticQuery,
            'semantic_query',
            self.handle_semantic_query
        )
        self.query_point_processor = QueryPointProcessor()
        self.default_strategy = self.query_point_processor.sim_1_sort_confs
        # testing
        # points = [Point(x=1.0, y=2.0, z=3.0), Point(x=4.0, y=5.0, z=6.0), Point(x=7.0, y=8.0, z=9.0)]
        # similarities = [0.8, 0.9, 0.7] 
        # service_names = ["service1", "service2", "service3"]
        # labels = ["label1", "label2", "label3"]
        # self.query_point_processor.update_values(points, similarities, service_names, labels)
        # points, similarities, service_names, labels = self.query_point_processor.sort_similarity()
        # self.get_logger().info(f"Points: {points}")
        # self.get_logger().info(f"Similarities: {similarities}")
        # self.get_logger().info(f"Service Names: {service_names}")
        # self.get_logger().info(f"Labels: {labels}")

    def handle_semantic_query(self, request, response):
        similarity_threshold = request.similarity_threshold_rad
        object_name = request.object_name
        self.get_logger().info("Received a query request")
        self.get_logger().info("Start sending socket message")
        self.send_text("query")
        self.wait_handshake("send_object")
        self.send_object_name(object_name)
        self.wait_handshake("send_sim")
        self.send_double(similarity_threshold)
        list_of_results = self.receive_semantic_map_results()
        # print(list_of_results)
        self.wait_handshake("done")

        corresponding_semantic_service = []
        similarities = []
        points = []
        labels = []
        confs = []
        for i in range(len(list_of_results)):
            if len(list_of_results[i].labels)==0:
                for j in range(len(list_of_results[i].similarities)):
                    corresponding_semantic_service.append(list_of_results[i].service_name)
                    similarities.append(list_of_results[i].similarities[j])
                    points.append(list_of_results[i].points[j])
                    labels.append("")
                    confs.append(-1)
            else:
                for j in range(len(list_of_results[i].labels)):
                    corresponding_semantic_service.append(list_of_results[i].service_name)
                    similarities.append(list_of_results[i].similarities[j])
                    points.append(list_of_results[i].points[j])
                    labels.append(list_of_results[i].labels[j])
                    confs.append(list_of_results[i].confs[j])
        print(corresponding_semantic_service)
        print(similarities)
        print(points)
        print(labels)
        print(confs)
        self.get_logger().info("Finish getting socket message")
        self.query_point_processor.update_values(points, similarities, corresponding_semantic_service, labels, confs)
        points, similarities, corresponding_semantic_service, labels, confs = self.default_strategy()
        print(corresponding_semantic_service)
        print(similarities)
        print(points)
        print(labels)
        print(confs)
        response.corresponding_semantic_service = corresponding_semantic_service
        response.similarities = similarities
        response.points = points
        response.labels = labels
        response.confs = confs
        self.get_logger().info("Returning Response")
        return response
    
    def receive_semantic_map_results(self):
        num_of_maps = struct.unpack('<L', self.sock.recv(4))[0]
        self.get_logger().info(f"Number of maps: {num_of_maps}")
        list_of_results = []
        for i in range(num_of_maps):
            data_size = struct.unpack('<L', self.sock.recv(4))[0]
            print("data_size", data_size)
            data = b""
            while len(data) < data_size:
                packet = self.sock.recv(data_size - len(data))
                if not packet:
                    break
                data+=packet
            list_of_results.append(rclpy.serialization.deserialize_message(data, ObjectSem))
            print(list_of_results)
            self.send_text("send_next")
        return list_of_results
    
    def send_text(self, text):
        try:
            self.get_logger().info(f"Sending text: {text}")
            data = text.encode()
            self.sock.sendall(data)
        except Exception as e:
            self.get_logger().error(f"Failed to send text: {e}")
    
    def send_object_name(self, object_name):
        try:
            self.get_logger().info(f"Sending object name: {object_name}")
            data = object_name.encode()
            self.sock.sendall(struct.pack('<L',len(data)) + data)
        except Exception as e:
            self.get_logger().error(f"Failed to send object name: {e}")

    def send_double(self, float_threshold):
        try:
            self.get_logger().info(f"Sending double threshold: {float_threshold}")
            data = struct.pack('d', float_threshold)
            self.sock.sendall(data)
        except Exception as e:
            self.get_logger().error(f"Failed to send double threshold: {e}")

    def socket_setup(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.connect_to, self.port_num))
        self.get_logger().info(f"Connected to {self.connect_to} on port {self.port_num}")   
    
    def wait_handshake(self, handshake_msg="handshake"):
        self.get_logger().info(f"Waiting for handshake {handshake_msg}")
        handshake = self.sock.recv(1024).decode()
        if handshake != handshake_msg:
            self.get_logger().error("Handshake failed")
            self.get_logger().error(f"Received: {handshake}, expect: {handshake_msg}")
            return False
        else:
            self.get_logger().info("Handshake successful")
            return True

    def close_socket(self):
        self.sock.close()

def main(args=None):
    rclpy.init(args=args)
    socket_sender = SocketSender(connect_to='fyp')
    print('connected')
    executor = MultiThreadedExecutor()
    executor.add_node(socket_sender)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        socket_sender.close_socket()
        socket_sender.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()