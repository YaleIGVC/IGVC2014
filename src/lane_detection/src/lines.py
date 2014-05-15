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

        image = frame[79:870, 545:1556]
        # cv2.imshow('original_image', image)

        # minColor = np.array([0, 0, 200],np.uint8)
        # maxColor = np.array([180, 255, 255],np.uint8)

        # hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # frame_threshed = cv2.inRange(hsv_img, minColor, maxColor)
        # cv2.imshow('thresed', frame_threshed)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # cv2.imwrite('output2.jpg', frame_threshed)
        # exit(0)
        # image = cv2.multiply(image,np.array([3.0]))
        # cv2.imshow('original_image2', image)
        # exit(0)
        # pdb.set_trace()
        gray_image_blue_channel, g, r = cv2.split(image)
        # r = cv2.multiply(r, np.array([0.0]))
        # cv2.imshow('merged', cv2.merge((gray_image_blue_channel, g, r)))
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # blur = cv2.GaussianBlur(gray_image_blue_channel,(11,11),0)
        blur = cv2.medianBlur(gray_image_blue_channel, 19)

        ret,thresh = cv2.threshold(blur,100,255,cv2.THRESH_TOZERO)
        kernel = np.ones((5,5),np.uint8)
        # erosion = cv2.erode(thresh, kernel, iterations = 1)
        morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        # minLineLength = 100
        # maxLineGap = 10
        # lines = cv2.HoughLinesP(morph,1,np.pi/180,100,minLineLength,maxLineGap)
        # for x1,y1,x2,y2 in lines[0]:
        #     cv2.line(image,(x1,y1),(x2,y2),(0,255,0),2)

        # cv2.imshow('original_image', image)
        # cv2.imshow('original_image', image)
        # cv2.imshow('gray_image', gray_image_blue_channel)
        # cv2.imshow('blurred', blur)
        # cv2.imshow('threshholded', thresh)
        # cv2.imshow('eroded', morph)
        # cv2.imshow('original_image', image)

        # (thresh, im_bw) = cv2.threshold(morph, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        thresh = 50
        im_bw = cv2.threshold(morph, thresh, 255, cv2.THRESH_BINARY)[1]
        dist_transform = cv2.distanceTransform(im_bw,cv2.cv.CV_DIST_L2,5)
        # cv2.imwrite('output2.jpg', im_bw)
        # contours,hierarchy = cv2.findContours(im_bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
