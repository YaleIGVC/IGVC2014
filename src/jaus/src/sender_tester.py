import socket

Robot_UDP_IP = "192.168.1.110"
MY_UDP_IP = ""
UDP_PORT = 3794

sock = socket.socket(socket.AF_INET,
                     socket.SOCK_DGRAM)
sock.bind((MY_UDP_IP, UDP_PORT))
while True:
    command = raw_input("JTC: ")
    sock.sendto(command, (Robot_UDP_IP, UDP_PORT))

    response, addr = sock.recvfrom(1024)
    print "Robot: ", response
