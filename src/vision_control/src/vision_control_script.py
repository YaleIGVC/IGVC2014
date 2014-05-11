#!/usr/bin/env python
import sys
import roslib
import rospy
import tf
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Transform
from frame_grabber_node.msg import ImageWithTransform
from flagmaster.msg import detectedflags
from vision_control.msg import detectedvision
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class flagmaster():
    def __init__(self):

        #Set stream to subscribe to
        if(len(sys.argv) > 1):
            camstring = sys.argv[1]
        else:
            camstring = "/raw_image"

        if(len(sys.argv) > 2):
            linestring = sys.argv[2]
        else:
            linetring = "/detected_lanes"

        if(len(sys.argv) > 3):
            flagstring = sys.argv[3]
        else:
            flagstring = "/detected_flags"

        self.node_name = "vision_control"

        self.newflag = False
        self.newlane = False
        #self.newimg = False

        self.blueimg = Image()
        self.redimg = Image()
        self.laneimg = Image()

        self.itf = Transform()

        self.finpub = rospy.Publisher("/flags_and_lanes", detectedvision)
        self.backpub = rospy.Publisher("/image_for_cv", Image)

        rospy.init_node(self.node_name)



        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        # Create the OpenCV display window for the RGB image
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        cv.MoveWindow(self.cv_window_name, 25, 75)


        # Create the cv_bridge object
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(camstring, ImageWithTransform, self.image_callback)
        self.flag_sub = rospy.Subscriber(flagstring, detectedflags, self.flag_callback)
        self.lane_sub = rospy.Subscriber(linestring, Image, self.lane_callback)


        rospy.loginfo("Waiting for camera, flag and lane topics...")

    def flag_callback(self, flag_images):

        self.blueimg = flag_images.blueflags
        self.redimg = flag_images.redflags

        self.newflag = True

        if self.newlane:
            pubdet = detectedvision()
            pubdet.redflags = self.redimg
            pubdet.blueflags = self.blueimg
            pubdet.lanes = self.laneimg
            pubdet.all = process_images(self.blueimg, self.redimg, self.laneimg)
            pubdet.tf = self.itf
            self.finpub.publish(pubdet)
            self.newflag = False
            self.newlane = False

    def lane_callback(self, lane_image):

        self.laneimg = lane_img

        self.newlane = True

        if self.newflag:
            pubdet = detectedvision()
            pubdet.redflags = self.redimg
            pubdet.blueflags = self.blueimg
            pubdet.lanes = self.laneimg
            pubdet.all = process_images(self.blueimg, self.redimg, self.laneimg)
            pubdet.tf = self.itf
            self.finpub.publish(pubdet)
            self.newflag = False
            self.newlane = False

    def image_callback(self, ros_image):

        if 1 not in (self.newblue, self.newred, self.newlane):
            self.backpub.publish(ros_image.image)
            self.itf = ros_image.tf
          
    def process_images(self, img1, img2, img3):
        try:
            imgcv1 = self.bridge.imgmsg_to_cv2(img1, "bgr8")
        except CvBridgeError, e:
            print e

        try:
            imgcv2 = self.bridge.imgmsg_to_cv2(img2, "bgr8")
        except CvBridgeError, e:
            print e

        try:
            imgcv3 = self.bridge.imgmsg_to_cv2(img3, "bgr8")
        except CvBridgeError, e:
            print e

        ibtw = cv2.bitwise_and(imgcv1,imgcv2, mask= mask1)

        res = cv2.bitwise_and(ibtw,imgcv3, mask= mask2)

        try:
            resrosimg = self.bridge.cv2_to_imgmsg(res, "bgr8")
        except CvBridgeError, e:
            print e
        
        return resrosimg
    
    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()   
    
def main(args):       
    try:
        flagmaster()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision controller node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
