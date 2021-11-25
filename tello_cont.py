import socket

class Tello:

    host = ''
    port = 9000
    locaddr = (host, port)

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tello_address = ('192.168.10.1', 8889)

    sock.bind(locaddr)

    # Socket通信による操作を有効化
    sock.sendto(b"command", tello_address)

    def send_cmd(self, cmd):
        self.sock.sendto(cmd.encode(encoding="utf-8"), self.tello_address)

    def close(self):
        self.sock.close()
