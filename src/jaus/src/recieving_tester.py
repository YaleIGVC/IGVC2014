import socket
 
My_UDP_IP = ""
JTC_UDP_IP = "192.168.1.127"
UDP_PORT = 3794
 
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

sock.bind((My_UDP_IP, UDP_PORT))

def jaus_switch_main(cmd):
	if cmd == 'hello_world':
		print "HEY BABEE"

while True:
    command, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    jaus_switch_main(command)
    print "JTC: ", command

    response = raw_input("Robot: ")
    sock.sendto(response, (JTC_UDP_IP, UDP_PORT))
