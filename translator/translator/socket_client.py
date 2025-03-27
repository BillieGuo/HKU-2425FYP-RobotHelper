'''
    This is a simple socket client that sends a message to the server and waits for a response.
    It shall be used to communicate with the robot-helper server, and can be one another machine.
'''

import socket
import struct
import time


class SocketClient:
    def __init__(self, host='robot-helper', port=7000):
        self.host = host
        self.port = port

    def connect(self):
        try:
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client.connect((self.host, self.port))
            return True
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, ConnectionRefusedError):
            print('Server is down, looping to reconnect')
            return False
        
    def send(self, msg):
        if not hasattr(self, 'client') or self.client.fileno() == -1:
            success = self.connect()
            while not success:
                success = self.connect()
                
        try:
            self.client.send(msg.encode())
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, ConnectionRefusedError):
            # print('Server is down, looping to reconnect')
            self.connect()
            return
        return self.client.recv(1024)
                
    def __del__(self):
        self.client.close()

def main():
    cnt = 0
    test = SocketClient(host='robot-helper', port=7000)
    while True:
        cnt += 1
        # msg = 'Hello World' + str(cnt)
        msg = input('Enter message: ')
        print(test.send(msg))
        time.sleep(1)
    
if __name__ == '__main__':
    main()