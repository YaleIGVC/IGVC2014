#!/usr/bin/env python

import serial
import os
import rospy
import time


portf = os.popen('echo $ARDUINO_SERIAL_PORT')

port = portf.read().strip('\n')

ser = serial.serial(port, 9600)

while not rospy.is_shutdown():
    if (['move_base_goal','PoseStamped'] in rospy.get_published_topics()):
        # TODO: Write to Arduino so it knows for how long to turn off light
        ser.write('O') # Tell the arduino to turn off the light for 750 ms
        time.sleep(1.5)


