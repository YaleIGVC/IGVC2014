#!/usr/bin/env python

from geometry_msgs.msg import Transform, Pose, Point, Quaternion, PoseStamped, PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from LatLongUTMconversion import LLtoUTM
import rospy
import time
import tf
import sys
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import roslib

class GoalScheduler():
    def __init__(self):
        rospy.init_node("goal_scheduler")

        #ROS
	self.listener = tf.TransformListener()
        self.pubtopic = rospy.Publisher("/move_base_goal", PoseStamped)
        rospy.on_shutdown(self.cleanup)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        currentgoalnum = 0

        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']

        #self.toleranceradius = 2

        #Read goals
        with open('goals.txt') as f:
            goals = f.read().splitlines()

        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
            cgoallatlon = goals[currentgoalnum].strip().split(",")
            clat = float(cgoallatlon[0])
            clon = float(cgoallatlon[1])
            utm_zone, easting, northing = LLtoUTM(23, clat, clon)

            tpoint = Point()
            tquat = Quaternion()
            tpose = Pose()
            tpsm = PoseStamped()
            theader = Header()
            theader.seq = currentgoalnum
            theader.stamp = rospy.Time(0)
            theader.frame_id = "map"
            tpoint.x = easting
            tpoint.y = northing
            tpoint.z = 0
            #tquat = tf.transformations.quaternion_from_euler(0,0,0)
            tpose.position = tpoint
            tpose.orientation = tquat
            tpsm.pose = tpose
            tpsm.header = theader 

            #self.goal = MoveBaseGoal()
            #self.goal.target_pose = tpsm
            #self.goal.target_pose.header = theader
            #self.move_base.send_goal(self.goal)
            rospy.loginfo("goal set")
            self.pubtopic.publish(tpsm)
            #finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

            #Check for success or failure
            #if not finished_within_time:
            #    self.move_base.cancel_goal()
            #    rospy.loginfo("Timed out achieving goal")
            #else:
            #    state = self.move_base.get_state()
            #    if state == GoalStatus.SUCCEEDED:
            #        currentgoalnum = currentgoalnum + 1
            #        rospy.loginfo("Goal succeeded!")
            #        rospy.loginfo("State:" + str(state))
            #    else:
            #      rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))

            #self.pubtopic.publish(self.goal)

    def cleanup(self):
        print "Shutting down goal scheduler."
        self.move_base.cancel_goal()
        rospy.sleep(2)


if __name__ == '__main__':
    GoalScheduler()
    rospy.spin()
