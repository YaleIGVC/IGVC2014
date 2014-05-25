#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError 
from sensor_msgs.msg import Image, CameraInfo 
from geometry_msgs.msg import Transform, Pose, Point, Quaternion, PoseStamped, PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
import rospy
import time
import tf

class GoalScheduler():
    def __init__(self):
        rospy.init_node("goal_scheduler")

        if(len(sys.argv) > 1):
            gpsstring = sys.argv[1]
        else:
            gpsstring = "/odom_utm"

        #ROS
        self.pubtopic = rospy.Publisher("/goal_coords", PoseStamped)
        rospy.on_shutdown(self.cleanup)

        self.gpssub = rospy.Subscriber(gpsstring, Odomoetry, self.goalie)

        self.currentgoalnum = 0
        self.firstrun = True
        self.initx = 0
        self.inity = 0

        self.seqcounter = 0

        self.toleranceradius = 2

        #Read goals
        with open('goals.txt') as f:
            self.goals = f.read().splitlines()

    def goalie(gpsdata):

        if(self.firstrun):
            self.initx = Odomoetry.pose.pose.position.x
            self.inity = Odomoetry.pose.pose.position.y
            self.firstrun = False

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

        ogps = goals[self.currentgoalnum].strip().split(",")



        if((abs(xcoord - float(ogps[0])) < self.toleranceradius) and (abs(xcoord - float(ogps[1])) < self.toleranceradius)):
            print "Hit goal"
            self.currentgoalnum = self.currentgoalnum + 1
            self.seqcounter = self.seqcounter + 1

            #compute new goal

            ncgps = goals[self.currentgoalnum].strip().split(",")

            tpoint.x = float(ncgps[0]) - self.initx
            tpoint.y = float(ncgps[1]) - self.inity

            theader.seq = self.seqcounter
            theader.stamp = rospy.Time(0)
            theader.frame_id = "1"

            tpose.orientation = tquat
            tpsm.pose = tpose
            tpsm.header = theader

            # Publish new goal coords
            self.pubtopic.publish(tpsm)

    def cleanup(self):
        print "Shutting down goal scheduler."


if __name__ == '__main__':
    GoalScheduler()
    rospy.spin()
