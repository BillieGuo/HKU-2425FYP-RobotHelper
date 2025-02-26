import socket
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


def start_server():
    # Create a socket object
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Get local machine name
    port = 6000

    # Bind to the port
    server_socket.bind(("0.0.0.0", port))

    # Start listening for incoming connections
    server_socket.listen(5)

    print("Server is listening...")

    while True:
        # Establish a connection
        client_socket, addr = server_socket.accept()
        print(f"Got a connection from {addr}")

        # Receive data from the client
        data = client_socket.recv(1024)
        msg_deserialized = deserialize_message(data, message_type=String)
        print(f"Received data: {msg_deserialized}")
        # data = client_socket.recv(1024).decode("utf-8")
        message = msg_deserialized.data
        # print(f"Received data: {data}")

        # Send a response to the client
        response = f"Data received: {message}"
        client_socket.send(response.encode("utf-8"))

        # Close the connection
        client_socket.close()


if __name__ == "__main__":
    start_server()
