#!/usr/bin/env python
import sys
import roslib
import rospy
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from frame_grabber_node.msg import ImageWithTransform
import numpy as np


class flagmaster():
    def __init__(self):

        camstring = "/image_for_cv"

        self.node_name = "flagmasteraaa"
        self.output_pub = rospy.Publisher("/detected_flags", Image)

        rospy.init_node(self.node_name)

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Subscribe to the camera image
        self.image_sub = rospy.Subscriber(camstring, ImageWithTransform, self.image_callback)

        rospy.loginfo("Waiting for image topic...")

    def image_callback(self, ros_image_wtf):
        # Use cv_bridge() to convert the ROS image to OpenCV format

        ros_image = ros_image_wtf.image
        
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e

        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        frame = np.array(frame, dtype=np.uint8)

        # Process the frame using the process_image() function
        processedimgs = self.process_image(frame)

        nvimg_blue = cv2.cvtColor(processedimgs['blue'], cv2.cv.CV_GRAY2BGR)
        nvimg_red = cv2.cvtColor(processedimgs['red'], cv2.cv.CV_GRAY2BGR)

        height, width, depth = nvimg_red.shape

        red = cv2.split(nvimg_red)[0]
        blue = cv2.split(nvimg_blue)[2]
        green = np.zeros((height, width), np.uint8)

        output_array = cv2.merge([blue, green, red])


        try:
            rosimgpub = self.bridge.cv2_to_imgmsg(output_array, "bgr8")
        except CvBridgeError, e:
            print e

        self.output_pub.publish(rosimgpub)
        rospy.loginfo("Published output")

    def process_image(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        lower_blue = np.array([115,50,50])
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
        print "Shutting down vision node."
        cv2.destroyAllWindows()

def main(args):
    try:
        flagmaster()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
