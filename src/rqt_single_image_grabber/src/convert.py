#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from datetime import datetime

if __name__ == '__main__':
    rospy.init_node("rqt_single_image_grabber")
    messagein = rospy.wait_for_message('/raw_image', Image)

    bridge = CvBridge()
    print "Attempting to grab image"
    try:
        frame = bridge.imgmsg_to_cv2(messagein, "bgr8")
    except CvBridgeError, e:
        print e
    cv2.imwrite(datetime.now().strftime('%Y-%m-%d-%H:%M:%S') + '-grabbed.jpg', frame)
    print "Success!"