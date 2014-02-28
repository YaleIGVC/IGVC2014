#!/usr/bin/env python
import pygame
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty
pygame.init()

pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

msg = """
Reading from the joystick!
"""

def getAxis():
    pygame.event.pump()
    left = joystick.get_axis(1)
    right = joystick.get_axis(4)
    #print (left,right)
    return (left,right)

speed = 10
turn = 5

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.init_node('teleop_twist_joystick')

    x = 0
    th = 0
    status = 0

    try:
        print msg
        print vels(speed,turn)
        while(1):
            joy = getAxis()
            x = joy[0]
            th = joy[1]
            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)
    except:
        print e

    finally:
       twist = Twist()
       twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
       twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
       pub.publish(twist)

       termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
