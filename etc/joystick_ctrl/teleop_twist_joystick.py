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
speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

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
			#key = getKey() # Commented out because otherwise program hangs until char
			key = '!'
			joy = getAxis()
			x = joy[0]
			th = joy[1]

			if key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15

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


