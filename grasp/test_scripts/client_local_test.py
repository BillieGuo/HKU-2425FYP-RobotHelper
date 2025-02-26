import socket
from rclpy.serialization import serialize_message
from std_msgs.msg import String


def start_client():
    # Create a socket object
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Get local machine name
    host = socket.gethostname()
    # host = "Carbonado"
    print(f"Host: {host}")
    port = 6000

    # Connect to the server
    client_socket.connect((host, port))

    # Send data to the server
    message = "Hello, Server!"
    # client_socket.send(message.encode("utf-8"))
    msg = String()
    msg.data = message
    data = serialize_message(msg)
    client_socket.send(data)

    # Receive response from the server
    response = client_socket.recv(1024).decode("utf-8")
    print(f"Received response: {response}")

    # Close the connection
    client_socket.close()


if __name__ == "__main__":
    start_client()
