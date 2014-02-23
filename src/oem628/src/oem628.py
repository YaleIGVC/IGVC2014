#!/usr/bin/env python
import rospy
import serial
import subprocess
from std_msgs.msg import String

# Initialize Serial comms
class Gps:
    def __init__(self):
        self.pub = rospy.Publisher('pos', String)
        rospy.init_node('oem628')

        self.BAUD = 9600
        p = subprocess.Popen(['./getusb.sh'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = p.communicate()
        usb = out[:-1]
        rospy.loginfo("Detected USB is /dev"+usb)
        self.ser = serial.Serial('/dev/'+usb,9600)

        # Start reporting GPS position
        self.ser.write("log bestposa ontime 1\n")

    def talker(self):
        while not rospy.is_shutdown():
            rawline = self.ser.readline()
            line = rawline.split(",")
            if len(line) > 11:
                lat = line[11]
                lon = line[12]
                str = lat + " " + lon
                rospy.loginfo(str)
                self.pub.publish(String(str))
            rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        gps = Gps()
        gps.talker()
    except rospy.ROSInterruptException:
        pass
