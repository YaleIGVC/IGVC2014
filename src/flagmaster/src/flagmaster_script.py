#!/usr/bin/env python
import sys
import roslib
import rospy
import tf
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from flagmaster.msg import detectedflags
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class flagmaster():
    def __init__(self):

        self.node_name = "flagmaster_flash"
        self.pub = rospy.Publisher("/detected_flags", detectedflags)

        rospy.init_node(self.node_name)

        #Set stream to subscribe to
        camstring = rospy.get_param('~camstream','/raw_image')

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        # Create the OpenCV display window for the RGB image
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        cv.MoveWindow(self.cv_window_name, 25, 75)


        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Subscribe to the camera image
        self.image_sub = rospy.Subscriber(camstring, Image, self.image_callback)

        rospy.loginfo("Waiting for image topic...")

    def image_callback(ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e

        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        frame = np.array(frame, dtype=np.uint8)
        
        # Process the frame using the process_image() function
        processedimgs = self.process_image(frame)


        nvimg = cv2.cvtColor(processedimgs['blue'], cv2.cv.CV_GRAY2BGR)

        try:
            blueimgpub = self.bridge.cv2_to_imgmsg(nvimg, "bgr8")
        except CvBridgeError, e:
            print e

        nvimg = cv2.cvtColor(processedimgs['red'], cv2.cv.CV_GRAY2BGR)

        try:
            redimgpub = self.bridge.cv2_to_imgmsg(nvimg, "bgr8")
        except CvBridgeError, e:
            print e

        pubfin = detectedflags()
        pubfin.redflags = redimgpub
        pubfin.blueflags = blueimgpub

        self.pub.publish(pubfin)
                       
        # Display the images.
        cv2.imshow(self.node_name + ' blue mask', processedimgs['blue'])
        cv2.imshow(self.node_name + ' red mask', processedimgs['red'])
        
        # Process any keyboard commands
        self.keystroke = cv.WaitKey(5)
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")
          
    def process_image(frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        lower_blue = np.array([80,50,50])
        upper_blue = np.array([130,255,255])

        # define range of red color in HSV
        lower_red = np.array([0,160,120])
        upper_red = np.array([10,255,255])

        # Thresholding
        bluemask = cv2.inRange(hsv, lower_blue, upper_blue)
        redmask = cv2.inRange(hsv, lower_red, upper_red)

        # Bitwise-AND mask and original image
        #res = cv2.bitwise_and(frame,frame, mask= mask)

        #cv2.imshow('frame',frame)
        #cv2.imshow('mask',mask)
        #cv2.imshow('res',res)
        
        return {'blue':bluemask, 'red':redmask}
    
    def cleanup(self):
        print "Shutting down flag detection node."
        cv2.destroyAllWindows()   
    
def main(args):       
    try:
        flagmaster()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down flag detection node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
