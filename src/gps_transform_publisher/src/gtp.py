#!/usr/bin/env python  
import roslib
from geometry_msgs.msg import Transform, Pose, Point, Quaternion, PoseStamped, PoseWithCovariance, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
import rospy
import tf
import time
import math
from vectornav.msg import ins

class Tpub():
    def __init__(self):
        rospy.init_node("gps_transform_publisher")

        #ROS
        rospy.on_shutdown(self.cleanup)

        self.gpssub = rospy.Subscriber("/odom_utm", Odometry, self.gpshandler)
        self.vnsub = rospy.Subscriber('/vectornav_vn200/ins',
                     ins,
                     self.transformatory)

        self.hasgps = False

        self.xcoord = 0
        self.ycoord = 0

	self.gz = 0
	self.hasyaw = False

    def gpshandler(self, gpsdata):

        if(not self.hasgps):

            self.xcoord = gpsdata.pose.pose.position.x
            self.ycoord = gpsdata.pose.pose.position.y

            self.hasgps = True

    def transformatory(self, imumsg):
        if(not self.hasyaw):
            self.gy = (imumsg.RPY.z)
            self.hasyaw = True

        if(self.hasgps):
            br = tf.TransformBroadcaster()
            br.sendTransform((self.xcoord, self.ycoord, 0),
                tf.transformations.quaternion_from_euler(0, 0, ((self.gy)*math.pi/180)),
                rospy.Time.now(),
                "odom_utm",
                "odom_combined")
            rospy.loginfo("transform published")

    def cleanup(self):
        print "Shutting down gps tranform publisher."


if __name__ == '__main__':
    Tpub()
    rospy.spin()
