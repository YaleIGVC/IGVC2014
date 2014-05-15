#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from datetime import datetime
import sys
import pdb

if __name__ == '__main__':
    if not sys.argv[1:]:
        print "Please specify a ros node!"
        quit(0)

    rospy.init_node("rqt_single_image_grabber")
    for arg in sys.argv[1:]:
        messagein = rospy.wait_for_message(arg, Image)

        bridge = CvBridge()
        print "Attempting to grab image"

        try:
            frame = bridge.imgmsg_to_cv2(messagein, "bgr8")
        except CvBridgeError, e:
            print e
        filename = datetime.now().strftime('%Y-%m-%d-%H:%M:%S') + '-grabbed-' + arg[1:].replace('/','-') + '.jpg'
        # pdb.set_trace()
        cv2.imwrite(filename, frame)
        print "Success!"
