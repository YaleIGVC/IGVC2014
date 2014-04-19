#!/usr/bin/env python

from pymba import *
import numpy as np 
from cv_bridge import CvBridge, CvBridgeError 
from sensor_msgs.msg import Image, CameraInfo 
import cv2
from cv2 import cv
import rospy
import time

# start Vimba
vimba = Vimba()
vimba.startup()

# get system object
system = vimba.getSystem()

# CvBridge
bridge = CvBridge()

#ROS
pubtopic = rospy.Publisher("/raw_image", Image)
rospy.init_node("frame_grabber")

# list available cameras (after enabling discovery for GigE cameras)
if system.GeVTLIsPresent:
    system.runFeatureCommand("GeVDiscoveryAllOnce")
    time.sleep(0.2)
cameraIds = vimba.getCameraIds()
for cameraId in cameraIds:
    rospy.loginfo('Camera ID: ' + cameraId)

# get and open a camera
camera0 = vimba.getCamera(cameraIds[0])
camera0.openCamera()
rospy.loginfo("Opened camera!")

# set the value of a feature
camera0.AcquisitionMode = 'SingleFrame'

# create new frames for the camera
frame0 = camera0.getFrame()    # creates a frame
frame1 = camera0.getFrame()    # creates a second frame

# announce frame
frame0.announceFrame()

# capture a camera image
camera0.startCapture()
frame0.queueFrameCapture()
camera0.runFeatureCommand('AcquisitionStart')
camera0.runFeatureCommand('AcquisitionStop')
frame0.waitFrameCapture()

# Use NumPy for fast image display
imgdata = np.ndarray(buffer = frame0.getBufferByteData(),
                               dtype = np.uint8,
                               shape = (frame0.height,
                                        frame0.width,
                                        1))

debayer = cv2.cvtColor(imgdata, cv.CV_BayerBG2BGR)
cv2.imshow('result', debayer), cv2.waitKey(0)
cv2.destroyAllWindows()

rosimgpub = bridge.cv2_to_imgmsg(debayer, "rgb8")

pubtopic.publish(rosimgpub)



# clean up after capture camera0.endCapture()
camera0.revokeAllFrames()

# close camera
camera0.closeCamera()
print "did I make it?"

# shutdown Vimba 
vimba.shutdown()
