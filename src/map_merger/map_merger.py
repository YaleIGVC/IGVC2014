#!/usr/bin/env python
import math
import rospy
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import tf.msg
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

def callback_laser_map(msg_in):

def callback_image_map(msg_in):

if __name__=='__main__':

    rospy.init_node('map_merger')