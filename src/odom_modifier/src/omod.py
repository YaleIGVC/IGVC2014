#!/usr/bin/env python  
import roslib
from geometry_msgs.msg import Transform, Pose, Point, Quaternion, PoseStamped, PoseWithCovariance, Vector3, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
import rospy
import tf
import time

from math import sin, cos, degrees, radians
from vectornav.msg import ins

class Tpub():
    def __init__(self):
        rospy.init_node("odom_modifier")

        #ROS
        rospy.on_shutdown(self.cleanup)

        self.gpssub = rospy.Subscriber("/odom_utm", Odometry, self.gpshandler)
        self.osub = rospy.Subscriber("/segway_rmp_node/odom", Odometry, self.odomhandler)
        self.pubtopic = rospy.Publisher("/odom_md", Odometry)
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

    def odomhandler(self, odommsg):
        if(self.hasyaw and self.hasgps):
            valtheta = radians(self.gy-90)

            xprime = (cos(valtheta)*(-odommsg.pose.pose.position.x)) - (sin(valtheta)*(-odommsg.pose.pose.position.y))
            yprime = (sin(valtheta)*(-odommsg.pose.pose.position.x)) + (cos(valtheta)*(-odommsg.pose.pose.position.y))

            #xprime = odommsg.pose.pose.position.x
            #yprime = odommsg.pose.pose.position.y

            odommsg.pose.pose.position.x = xprime + self.xcoord
            odommsg.pose.pose.position.y = yprime + self.ycoord
            odommsg.pose.pose.position.z = 0

            orientation_array = tf.transformations.quaternion_from_euler(0, 0, valtheta)
            odommsg.pose.pose.orientation.x = orientation_array[0]
            odommsg.pose.pose.orientation.y = orientation_array[1]
            odommsg.pose.pose.orientation.z = orientation_array[2]
            odommsg.pose.pose.orientation.w = orientation_array[3]

            xvprime = (cos(valtheta)*odommsg.twist.twist.linear.x) - (sin(valtheta)*odommsg.twist.twist.linear.y)
            yvprime = (sin(valtheta)*odommsg.twist.twist.linear.x) + (cos(valtheta)*odommsg.twist.twist.linear.y)

            odommsg.twist.twist.linear.x = xvprime;
            odommsg.twist.twist.linear.y = yvprime;

            self.pubtopic.publish(odommsg)

        

    def cleanup(self):
        print "Shutting down odom odom_modifier."


if __name__ == '__main__':
    Tpub()
    rospy.spin()
