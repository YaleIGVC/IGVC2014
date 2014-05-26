#!/usr/bin/env python
import cv2
import numpy as np
import time
import rospy
from sensor_msgs.msg import Image
from frame_grabber_node.msg import ImageWithTransform
from cv_bridge import CvBridge, CvBridgeError

class ImageHandler():
    def __init__(self): 
        self.output = rospy.Publisher("/lanes_and_flags", ImageWithTransform)
        self.bridge = CvBridge()
        rospy.Subscriber("/image_unwarp/output_video", ImageWithTransform, self.callback, queue_size=1, buff_size = 2**24)

    def callback(self, msg_in):
        output_msg = ImageWithTransform()
        output_msg.tf = msg_in.tf

        try:
            np_input_image = self.bridge.imgmsg_to_cv2(msg_in.image, "bgr8")
        except CvBridgeError, e:
            print e

        #np_input_image = cv2.flip(np_input_image, -1)

        green_lane_img = self.lane_handler(np_input_image)
        red_flag_img, blue_flag_img = self.flag_handler(np_input_image)

        output_array = cv2.merge([blue_flag_img, green_lane_img, red_flag_img])
        
        (image_width, image_height, _temp_) = output_array.shape
        resize_width = 100
        resize_height = resize_width * (image_height/image_width)
        output_array = cv2.resize(output_array, (resize_width, resize_height))
    
        try:
            rosimgpub = self.bridge.cv2_to_imgmsg(output_array, "bgr8")
        except CvBridgeError, e:
            print e

        output_msg.image = rosimgpub
        self.output.publish(output_msg)
        rospy.loginfo("Sent image!")

    def lane_handler(self, image):
        # image = frame[79:870, 545:1556]

        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #image = cv2.resize(image, (image.shape[1] / 2, image.shape[0] / 2))

        org_brwn_min = np.array([0, 0, 0], np.uint8)
        org_brwn_max = np.array([62, 255, 255], np.uint8)

        org_brwn_thresh = cv2.inRange(image, org_brwn_min, org_brwn_max)
        org_brwn_thresh = 255 - cv2.cvtColor(org_brwn_thresh, cv2.COLOR_GRAY2RGB)
        new_hsv = np.bitwise_and(org_brwn_thresh, image)

        # white_min = np.array([0, 0, 140], np.uint8)
        # white_max = np.array([180, 70, 255], np.uint8)

        # white_thresh = cv2.inRange(new_hsv, white_min, white_max)
        # white_thresh = cv2.cvtColor(white_thresh, cv2.COLOR_GRAY2RGB)
        # new_hsv = np.bitwise_and(white_thresh, new_hsv)

        image = cv2.cvtColor(new_hsv, cv2.COLOR_HSV2BGR)

        gray_image_blue_channel, g, r = cv2.split(image)

        blur = cv2.medianBlur(gray_image_blue_channel, 11)

        ret,thresh = cv2.threshold(blur,100,255,cv2.THRESH_TOZERO)
        kernel = np.ones((5,5),np.uint8)
        morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        thresh = 50
        im_bw = cv2.threshold(morph, thresh, 255, cv2.THRESH_BINARY)[1]

        new_bgr= cv2.cvtColor(im_bw,cv2.COLOR_GRAY2RGB)

        width, height, depth = new_bgr.shape
        green = np.zeros((width, height), np.uint8)
        #green = cv2.split(new_bgr)[1]

        return green

    def flag_handler(self, np_image):
        # Process the frame using the process_image() function
        hsv = cv2.cvtColor(np_image, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        lower_blue = np.array([115,50,50])
        upper_blue = np.array([130,255,255])

        # define range of red color in HSV
        lower_red = np.array([0,160,120])
        upper_red = np.array([10,255,255])

        # Thresholding
        bluemask = cv2.inRange(hsv, lower_blue, upper_blue)
        redmask = cv2.inRange(hsv, lower_red, upper_red)

        #contoursred, hierarchyred = cv2.findContours(redmask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #contoursblue, hierarchyblue = cv2.findContours(bluemask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        nvimg_blue = cv2.cvtColor(bluemask, cv2.cv.CV_GRAY2BGR)
        nvimg_red = cv2.cvtColor(redmask, cv2.cv.CV_GRAY2BGR)

        red = cv2.split(nvimg_red)[0]
        blue = cv2.split(nvimg_blue)[2]
        return red, blue

if __name__=='__main__':
    rospy.init_node("obstacle_processor")
    ImageHandler()
    rospy.spin()
