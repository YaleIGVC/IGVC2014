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

Resolution = .05 #meters per cell
Width = 2000 #100 meters * 20 cells per meter
Height = 2000 #100 meters * 20 cells per meter
MinLaserRange = .2 #minimum distance for obstacles to be considered

def callback_laser(msg_in):
    global pub_map
    global tf_listener
    global Origin
    global Map
    global mapData

    angle_min = msg_in.angle_min
    angle_max = msg_in.angle_max
    angle_increment = msg_in.angle_increment
    max_range = msg_in.range_max
    laser_ranges = msg_in.ranges

    try:
        tf_listener.waitForTransform("odom_combined", "laser", rospy.Time(0), rospy.Duration(3.0))
        (trans,rot) = tf_listener.lookupTransform("odom_combined", "laser", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException) as e:
        print "odom_combined to laser tf lookup failure"

    angles = euler_from_quaternion(rot)

    quat = [Origin.orientation.x,
            Origin.orientation.y,
            Origin.orientation.z,
            Origin.orientation.w]

    originAngles = euler_from_quaternion(quat)

    for r in laser_ranges:
        x = 0
        y = 0
        if (not math.isnan(r)) and (r < max_range) and (r > MinLaserRange):
            x = trans[0] - Origin.position.x
            y = trans[1] - Origin.position.y

            x = x + (r*math.cos(angles[2]+angle_min-originAngles[2]))
            y = y + (r*math.sin(angles[2]+angle_min-originAngles[2]))

            x = int(round(x*(1/Resolution)))
            y = int(round(y*(1/Resolution))) 

            if x<0 or x>(Width-1) or y<0 or y>(Height-1):
                print "Outside map bounds!!!!"

            else:
                index = ((y*Width)+x)
                if mapData[index] < 100:
                    #mapData[index] = mapData[index] + 5
                    mapData[index] = 100
            
        angle_min = angle_min + angle_increment 

    Map.header.stamp = rospy.get_rostime()
    Map.header.frame_id = 'map'
    Map.info.map_load_time = rospy.get_rostime()
    Map.data = mapData

    rospy.loginfo("Publishing a map")
    pub_map.publish(Map)

if __name__=='__main__':
    global pub_map
    global tf_listener
    global Origin
    global Map
    global mapData

    rospy.init_node('mapper')
    time = rospy.get_rostime()

    tf_listener = tf.TransformListener()
    
    Origin = Pose()
    try:
         tf_listener.waitForTransform("odom_combined", "base_link", rospy.Time(0), rospy.Duration(3.0))
         (trans,rot) = tf_listener.lookupTransform("odom_combined", "laser", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException) as e:
        print "odom_combined to base_link tf lookup failure"

    angles = euler_from_quaternion(rot)
    rot = quaternion_from_euler(0.0, 0.0, angles[2])

    point = Point()
    point.x = trans[0] - ((Width/2)*.05)
    point.y = trans[1]- ((Height/2)*.05)
    point.z = trans[2]

    rotation = Quaternion()
    rotation.x=rot[0]
    rotation.y=rot[1]
    rotation.z=rot[2]
    rotation.w=rot[3]

    Origin.position = point
    Origin.orientation = rotation

    metaData = MapMetaData()
    metaData.map_load_time = rospy.get_rostime()
    metaData.width = Width 
    metaData.height = Height
    metaData.resolution = Resolution
    metaData.origin = Origin 

    Map = OccupancyGrid()
    Map.info = metaData
    mapData = [0]*(Width*Height)

    
    pub_map = rospy.Publisher("/map", OccupancyGrid)
    
    rospy.Subscriber("/scan", LaserScan, callback_laser)
    rospy.loginfo("init")
    rospy.spin()

