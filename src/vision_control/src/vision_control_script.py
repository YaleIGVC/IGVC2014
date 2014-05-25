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
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class mvc():
    def __init__(self):

        #Set stream to subscribe to
        camstring = "/image_unwarp/output_video"

        linestring = "/detected_lanes"

        flagstring = "/detected_flags"

        self.node_name = "vision_control"

        self.newflag = False
        self.newlane = False

        self.flagimg = Image()
        self.laneimg = Image()

        self.itf = Transform()

        self.cv_pub = rospy.Publisher("/image_for_cv", Image)
        self.output_pub = rospy.Publisher("/lanes_and_flags", ImageWithTransform)

        rospy.init_node(self.node_name)

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(camstring, ImageWithTransform, self.image_callback)
        self.flag_sub = rospy.Subscriber(flagstring, Image, self.flag_callback)
        self.lane_sub = rospy.Subscriber(linestring, Image, self.lane_callback)

        rospy.loginfo("Waiting for camera, flag and lane topics...")

    def flag_callback(self, flag_image):

        self.flagimg = flag_image
        self.newflag = True

        if self.newlane:
            pubdet = Image()
            pubdet = process_images(self.laneimg, self.flagimg)
            self.output_pub.publish(pubdet.all)
            pubdet.tf = self.itf
            self.newflag = False
            self.newlane = False

    def lane_callback(self, lane_image):

        self.laneimg = lane_img

        self.newlane = True

        if self.newflag:
            pubdet = Image()
            pubdet.redflags = self.redimg
            pubdet.blueflags = self.blueimg
            pubdet.lanes = self.laneimg
            pubdet.all = process_images(self.laneimg, self.blueimg, self.redimg)
            self.output_pub.publish(pubdet.all)
            pubdet.tf = self.itf
            self.newflag = False
            self.newlane = False

    def image_callback(self, ros_image):

        if 1 not in (self.newflag, self.newlane):
            self.cv_pub.publish(ros_image.image)
            self.itf = ros_image.tf
          
    def process_images(self, img1, img2):
        try:
            imgcv1 = self.bridge.imgmsg_to_cv2(img1, "bgr8")
        except CvBridgeError, e:
            print e

        try:
            imgcv2 = self.bridge.imgmsg_to_cv2(img2, "bgr8")
        except CvBridgeError, e:
            print e

        #ibtw = cv2.bitwise_or(imgcv1,imgcv2, mask= mask1)

        #res = cv2.bitwise_or(ibtw,imgcv3, mask= mask2)

        res = cv2.bitwise_or(imgcv1,imgcv2, mask=mask2)

        try:
            resrosimg = self.bridge.cv2_to_imgmsg(res, "bgr8")
        except CvBridgeError, e:
            print e
        
        return resrosimg
    
    def cleanup(self):
        print "Shutting down vision controller node."
        cv2.destroyAllWindows()   
    
def main(args):       
    try:
        mvc()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision controller node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
