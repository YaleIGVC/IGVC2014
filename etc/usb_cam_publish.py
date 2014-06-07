#!/usr/bin/env python

import rospy
import cv2
import signal
import sys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import tf
from frame_grabber_node.msg import ImageWithTransform


def s_hand(signal, frame):
    sys.exit(0)

rospy.init_node("usb_camera_publish")
listener = tf.TransformListener()
imgpub = rospy.Publisher("/raw_image_with_tf", ImageWithTransform)
bridge = CvBridge()

vc = cv2.VideoCapture(0)

if vc.isOpened(): # try to get the first frame
    signal.signal(signal.SIGINT, s_hand)
    rval, frame = vc.read()
else:
    rval = False

ImagewithTF = ImageWithTransform()

while rval:
    rval, frame = vc.read()
    try:
	img = bridge.cv2_to_imgmsg(frame, "bgr8")
    except CvBridgeError, e:
        print e
    
    xform = Transform()
    self.listener.waitForTransform("odom_combined", "camera", rospy.Time(0), rospy.Duration(3.0))
    (trans,rot) = self.listener.lookupTransform('/odom_combined', '/camera', rospy.Time(0))
    xform.translation.x = trans[0]
    xform.translation.y = trans[1]
    xform.translation.z = trans[2]
    xform.rotation.x = rot[0]
    xform.rotation.y = rot[1]
    xform.rotation.z = rot[2]
    xform.rotation.w = rot[3]

    ImagewithTF.image = img
    ImagewithTF.tf = xform
    
    imgpub.publish(ImagewithTF)

 
