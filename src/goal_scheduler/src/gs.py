#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError 
from sensor_msgs.msg import Image, CameraInfo 
from geometry_msgs.msg import Transform, Pose, Point, Quaternion, PoseStamped, PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
import rospy
import time
import tf
import sys

class GoalScheduler():
    def __init__(self):
        rospy.init_node("goal_scheduler")

        if(len(sys.argv) > 1):
            gpsstring = sys.argv[1]
        else:
            gpsstring = "/odom_utm"

        #ROS
	    self.listener = tf.TransformListener()
        self.pubtopic = rospy.Publisher("/goal_coords", PoseStamped)
        rospy.on_shutdown(self.cleanup)

        self.gpssub = rospy.Subscriber(gpsstring, Odometry, self.goalie)

        self.currentgoalnum = 0
        self.firstrun = True
        self.initx = 0
        self.inity = 0

        self.seqcounter = 0

        self.toleranceradius = 2

        #Read goals
        with open('goals.txt') as f:
            self.goals = f.read().splitlines()

    def goalie(self, gpsdata):

        if(self.firstrun):
            self.initx = gpsdata.pose.pose.position.x
            self.inity = gpsdata.pose.pose.position.y

        xform = Transform()
        tpoint = Point()
        tquat = Quaternion()
        tpose = Pose()
        tpsm = PoseStamped()
        theader = Header()

        self.listener.waitForTransform("odom_combined", "base_link", rospy.Time(0), rospy.Duration(3.0))
        (trans,rot) = self.listener.lookupTransform('/odom_combined', '/base_link', rospy.Time(0))



        xcoord = trans[0]
        ycoord = trans[1]
        tpoint.z = trans[2]
        tquat.x = rot[0]
        tquat.y = rot[1]
        tquat.z = rot[2]
        tquat.w = rot[3]

        ogps = self.goals[self.currentgoalnum].strip().split(",")



        if(self.firstrun or ((abs(xcoord - float(ogps[0])) < self.toleranceradius) and (abs(xcoord - float(ogps[1])) < self.toleranceradius))):
            print "Hit goal"
            self.currentgoalnum = self.currentgoalnum + 1
            self.seqcounter = self.seqcounter + 1

	        self.firstrun = False

            #compute new goal

            ncgps = self.goals[self.currentgoalnum].strip().split(",")

            tpoint.x = float(ncgps[0]) - self.initx
            tpoint.y = float(ncgps[1]) - self.inity

            theader.seq = self.seqcounter
            theader.stamp = rospy.Time(0)
            theader.frame_id = "1"

            tpose.orientation = tquat
	        tpose.position = tpoint
            tpsm.pose = tpose
            tpsm.header = theader
	        rospy.loginfo("new goal")

            # Publish new goal coords
            self.pubtopic.publish(tpsm)

    def cleanup(self):
        print "Shutting down goal scheduler."


if __name__ == '__main__':
    GoalScheduler()
    rospy.spin()
