import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf
from frame_grabber_node.msg import ImageWithTransform

def callback_unwarp(msg_in):
    global pub
    global bridge

    try:
        im = bridge.imgmsg_to_cv2(msg_in.image, "bgr8")
    except CvBridgeError, e:
        print e

    # Before changing these parameters, run `rqt_single_image_grabber` to grab a raw
    # chessboard image. Place the chessboard at the bottom center, and grab four corners.
    # Order of [x,y] points is clockwise starting from upper-left.

    # Corners of theoretical desired rectified square.
    img1_square_corners = np.float32([[248, 287], [392, 287], [392, 479],[248, 479]])
    # Corners of raw, unwarped chessboard.
    img2_quad_corners = np.float32([[258,339], [381,338], [392,479], [248,479]])

    h, mask = cv2.findHomography(img2_quad_corners, img1_square_corners)
    out = cv2.warpPerspective(im, h, (640, 480)) # 640 x 480 is the image width and height.

    try:
        rosimgpub = bridge.cv2_to_imgmsg(out, "bgr8")
    except CvBridgeError, e:
        print e

    output = ImageWithTransform()
    output.image = rosimgpub
    output.tf = msg_in.tf
    pub.publish(output)

if __name__=='__main__':
    rospy.init_node('homography_filter')
    global pub
    global bridge

    bridge = CvBridge()
    pub = rospy.Publisher("/image_unwarp/output_video", ImageWithTransform)
    rospy.Subscriber("/raw_image_with_tf", ImageWithTransform, callback_unwarp)
    rospy.loginfo("init")
    rospy.spin()
