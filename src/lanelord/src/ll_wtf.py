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


class lanelord():
    def __init__(self):

        camstring = rospy.get_param('~camstring','/image_unwarp/output_video')

        self.node_name = "lanelord"
        self.output_pub = rospy.Publisher("/detected_lanes", Image)

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
        ttf = ros_image_wtf.tf

        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e

        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        frame = np.array(frame, dtype=np.uint8)

        # Process the frame using the process_image() function
        processedimg = self.process_image(frame)


        try:
            rosimgpub = self.bridge.cv2_to_imgmsg(processedimg, "bgr8")
        except CvBridgeError, e:
            print e

        rosimagewtfpub = ImageWithTransform()
        rosimagewtfpub.image = rosimgpub
        rosimagewtfpub.tf = ttf

        self.output_pub.publish(rosimagewtfpub)
        rospy.loginfo("Published output")

    def process_image(self, img):

        lowThreshold = 250
        max_lowThreshold = 300
        ratio = 3
        kernel_size = 5

        img = cv2.resize(img, (img.shape[1] / 8, img.shape[0] / 8))
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        detected_edges = cv2.GaussianBlur(gray,(11,11),0)
        detected_edges = cv2.Canny(detected_edges,lowThreshold,lowThreshold*ratio,apertureSize = kernel_size)
        dst = cv2.bitwise_and(img,img,mask = detected_edges)  # just add some colours to edges from original image.
        cv2.imshow('edges (canny)',dst)

        return dst

    def cleanup(self):
        print "Shutting down lanelord node."
        cv2.destroyAllWindows()

def main(args):
    try:
        lanelord()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
