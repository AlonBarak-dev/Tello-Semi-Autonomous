from socket import *
import time

def main():
    serverSocket = socket(AF_INET, SOCK_DGRAM)
    serverSocket.bind(('', 8889))
    serverSocket.sendto('command'.encode(), ('192.168.10.1', 8889))
    print("Command sent")
    data, _ = serverSocket.recvfrom(128)
    if data.decode() != 'ok':
        raise RuntimeError('Tello rejected attempt to enter command mode')
    print(data.decode())

    data = ''

    serverSocket.sendto('battery?'.encode(), ('192.168.10.1', 8889))
    data, _ = serverSocket.recvfrom(128)
    print("Battery percentage:", data.decode())

    time.sleep(1)
    serverSocket.close()

if __name__ == '__main__':
    main()