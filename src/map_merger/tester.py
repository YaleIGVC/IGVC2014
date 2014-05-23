#!/usr/bin/env Python
import rospy
import time
from sensor_msgs.msg import Image
from frame_grabber_node.msg import ImageWithTransform
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import tf.msg
import tf

rospy.init_node('map_merger_tester')

img = cv2.imread('/home/inve14/IGVC2014/src/map_merger/fake_lines.jpg', 1)
bridge = CvBridge()
tf_listener  = tf.TransformListener()
my_img = ImageWithTransform()
my_img.image = bridge.cv2_to_imgmsg(img, "bgr8")



pub_merged_map = rospy.Publisher("/fake_lines", ImageWithTransform)

while True:
    tf_listener.waitForTransform("odom_combined", "base_link", rospy.Time(), rospy.Duration(3))
    
    (trans, rot) = tf_listener.lookupTransform("odom_combined", "base_link", rospy.Time())
    my_img.tf.rotation.x = rot[0]
    my_img.tf.rotation.y = rot[1]
    my_img.tf.rotation.z = rot[2]
    my_img.tf.rotation.w = rot[3]
    my_img.tf.translation.x = trans[0]
    my_img.tf.translation.y = trans[1]
    my_img.tf.translation.z = trans[2]
    
    pub_merged_map.publish(my_img)
    rospy.loginfo("Publishing fake lines")
    time.sleep(.2)
    



