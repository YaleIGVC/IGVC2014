#!/usr/bin/env python

from pymba import *
import numpy as np 
from cv_bridge import CvBridge, CvBridgeError 
from sensor_msgs.msg import Image, CameraInfo 
from geometry_msgs.msg import Transform
import cv2
from cv2 import cv
import rospy
import time
from frame_grabber.msg import ImageWithTransform

#class FrameGrabber():
    #def __init__(self):
        ## start Vimba
        #self.vimba = Vimba()
        #self.vimba.startup()

        ## get system object
        #system = self.vimba.getSystem()

        ## CvBridge
        #self.bridge = CvBridge()

        ## ROS Transforms
        #self.listener = tf.TransformListener()

        ##ROS
        #self.rawimgtopic = rospy.Publisher("/raw_image", Image)
        #self.tfimgtopic = rospy.Publisher("/raw_image_with_tf", Image)
        #rospy.init_node("frame_grabber")
        #rospy.on_shutdown(self.cleanup)

        ## list available cameras (after enabling discovery for GigE cameras)
        #if system.GeVTLIsPresent:
            #system.runFeatureCommand("GeVDiscoveryAllOnce")
            #time.sleep(0.2)
        #cameraIds = self.vimba.getCameraIds()
        #for cameraId in cameraIds:
            #rospy.loginfo('Camera ID: ' + cameraId)

        ## get and open a camera
        #self.camera = self.vimba.getCamera(cameraIds[0])
        #self.camera.openCamera()
        #rospy.loginfo("Opened camera!")

        ## set the feature values
        #self.camera.AcquisitionMode = 'SingleFrame'

        ## Start capture!!!
        #self.camera.startCapture()

        #self.frame0 = self.camera.getFrame()    # creates a frame
        ## announce frame
        #self.frame0.announceFrame()

    #def grabFrame(self):
        #rospy.loginfo("Grabbing frame!");
        #self.frame0.queueFrameCapture()
        ## create new frames for the camera

        ## capture a camera image
        #self.camera.runFeatureCommand('AcquisitionStart')
        #self.camera.runFeatureCommand('AcquisitionStop')
        #self.frame0.waitFrameCapture()

        ## Grab TF!!
        #xform = Transform()

        #self.listener.waitForTransform("odom_combined", "base_footprint", rospy.Time(0), rospy.Duration(3.0))
        #(xform.translation,xform.rotation) = self.listener.lookupTransform('/odom_combined', '/base_footprint', rospy.Time(0))

        ## Use NumPy for fast image display
        #imgdata = np.ndarray(buffer = self.frame0.getBufferByteData(),
                                       #dtype = np.uint8,
                                       #shape = (self.frame0.height,
                                                #self.frame0.width,
                                                #1))

        #debayer = cv2.cvtColor(imgdata, cv.CV_BayerGR2BGR)
        ##cv2.imshow('result', debayer), cv2.waitKey(0)
        ##cv2.destroyAllWindows()

        #try:
            #rosimgpub = self.bridge.cv2_to_imgmsg(debayer, "bgr8")
        #except CvBridgeError, e:
            #print e

        ## Assemble ImageWithTransform package
        #tfImg = ImageWithTransform()
        #tfImg.tf = xform
        #tfImg.image = rosimgpub

        ## Publish!!
        #self.rawimgtopic.publish(rosimgpub)
        #self.tfimgtopic.publish(rosimgpub)

    #def cleanup(self):
        ## clean up after capture self.camera.endCapture()
        #self.camera.revokeAllFrames()

        ## close camera
        #self.camera.closeCamera()

        ## shutdown Vimba
        #self.vimba.shutdown()

#if __name__ == '__main__':
    #f = FrameGrabber()
    #while not rospy.is_shutdown():
        #f.grabFrame()
    #rospy.spin()


