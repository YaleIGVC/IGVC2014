import socket
 
My_UDP_IP = ""
JTC_UDP_IP = "192.168.1.42"
UDP_PORT = 3794
 
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

sock.bind((My_UDP_IP, UDP_PORT))

while True:
    command, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print "JTC: ", command

    response = raw_input("Robot: ")
    sock.sendto(response, (JTC_UDP_IP, UDP_PORT))