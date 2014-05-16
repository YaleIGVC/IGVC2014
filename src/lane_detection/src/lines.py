#!/usr/bin/env python

import cv2
import numpy as np
import pdb
import time
import rospy
from sensor_msgs.msg import Image
from frame_grabber_node.msg import ImageWithTransform
from cv_bridge import CvBridge, CvBridgeError

class LaneDetector():
    def __init__(self):
        rospy.init_node("lane_detector")
        self.output = rospy.Publisher("/detected_lanes", Image)
        rospy.Subscriber("/image_unwarp/output_video", Image, self.callback)

        self.bridge = CvBridge()

    def callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        start_time = time.time()

        # image = frame[79:870, 545:1556]
        
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        image = cv2.resize(image, (image.shape[1] / 2, image.shape[0] / 2))

        # cv2.imshow('image', image)
        org_brwn_min = np.array([0, 0, 0], np.uint8)
        org_brwn_max = np.array([62, 255, 255], np.uint8)

        org_brwn_thresh = cv2.inRange(image, org_brwn_min, org_brwn_max)
        org_brwn_thresh = 255 - cv2.cvtColor(org_brwn_thresh, cv2.COLOR_GRAY2RGB)
        # pdb.set_trace()
        new_hsv = np.bitwise_and(org_brwn_thresh, image)
        # cv2.imshow('org_brwn_thresh', org_brwn_thresh)
        # cv2.imshow('new', new_hsv)

        # white_min = np.array([0, 0, 140], np.uint8)
        # white_max = np.array([180, 70, 255], np.uint8)

        # white_thresh = cv2.inRange(new_hsv, white_min, white_max)
        # white_thresh = cv2.cvtColor(white_thresh, cv2.COLOR_GRAY2RGB)
        # new_hsv = np.bitwise_and(white_thresh, new_hsv)
        # cv2.imshow('white_thresh', new_hsv)



        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # exit(0)


        image = cv2.cvtColor(new_hsv, cv2.COLOR_HSV2BGR)
        # cv2.imshow('new new image', image)

        # cv2.waitKey(0)

        gray_image_blue_channel, g, r = cv2.split(image)

        blur = cv2.medianBlur(gray_image_blue_channel, 11)

        ret,thresh = cv2.threshold(blur,100,255,cv2.THRESH_TOZERO)
        kernel = np.ones((5,5),np.uint8)
        morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        # cv2.imshow('im_bw', morph)

        thresh = 50
        im_bw = cv2.threshold(morph, thresh, 255, cv2.THRESH_BINARY)[1]


        new_bgr= cv2.cvtColor(im_bw,cv2.COLOR_GRAY2RGB)

        try:
            rosimgpub = self.bridge.cv2_to_imgmsg(new_bgr, "bgr8")
        except CvBridgeError, e:
            print e

        finaloutput = ImageWithTransform()
        # finaloutput.tf = ros_image.tf
        finaloutput.image = rosimgpub
        # print new_bgr
        self.output.publish(finaloutput.image)
        rospy.loginfo("Published lanes")
        print time.time() - start_time, "seconds"
if __name__=='__main__':
    LaneDetector()
    rospy.spin()
