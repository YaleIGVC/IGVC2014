import socket
import rospy

My_UDP_IP = ""
# THIS IS THE IP ADDRESS OF THE JAUS SYSTEM AT COMPETITION
JTC_UDP_IP = "192.168.1.42"
UDP_PORT = 3794 # JAUS UDP Port as specified in the manual

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

sock.bind((My_UDP_IP, UDP_PORT))

def jaus_switch_main(cmd, socket):
    if cmd == 'hello_world':
        response = "Hey!"
    elif cmd == 'Velocity':
        global PresenceVector
        response = PresenceVector[0] # Velocity X

    socket.sendto(response, (JTC_UDP_IP, UDP_PORT))

def odomCallback(msg_in):
    pass
def velocityCallback(msg_in):
    global PresenceVector
    PresenceVector[0] = 1
    PresenceVector[1] = 5
    PresenceVector[2] = 10

if __name__ == "__main__":
    rospy.init_node("jaus")
    rospy.Subscriber("/segway_rmp_node/odom",); # For x velocity data
    rospy.Subscriber("/Imu");                  # For rotation data
    rospy.Subscriber("/segway_rmp_node/odom");
    while True:
        command, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        jaus_switch_main(command,sock)
        print "JTC: ", command

        response = raw_input("Robot: ")
