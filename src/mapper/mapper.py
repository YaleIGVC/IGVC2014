#!/usr/bin/env python
import math
import rospy
import tf
from tf.transformations import euler_from_quaternion
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

def callback_laser(msg_in):
    global pub_map
    global tf_listener
    global Origin
    global Map
    global mapData


    angle_min = msg_in.angle_min
    angle_max = msg_in.angle_max
    angle_increment = msg_in.angle_increment
    ranges = msg_in.ranges

    try:
        tf_listener.waitForTransform("odom_combined", "laser", rospy.Time(0), rospy.Duration(3.0))
        (trans,rot) = tf_listener.lookupTransform("odom_combined", "laser", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException) as e:
        print "FailFail!"
        print e
    angles = euler_from_quaternion(rot)

    originAngles = euler_from_quaternion(Origin.orientation)

    for r in ranges:
        if not math.isnan(r):
            x = trans[0] - Origin.position.x
            y = trans[1] - Origin.position.y

            x = y + (r/math.cos(angles[2]+angle_min-originAngles[2]))
            y = y + (r/math.sin(angles[2]+angle_min-originAngles[2]

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

    Map.header.stamp = msg_in.header.stamp
    Map.header.frame_id = 'map'
    Map.data = mapData

    rospy.loginfo("publishing a map")
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
         tf_listener.waitForTransform("odom_combined", "laser", rospy.Time(0), rospy.Duration(3.0))
         (trans,rot) = tf_listener.lookupTransform("odom_combined", "laser", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException) as e:
        print "Fail!"
        print e
    point = Point()
    point.x = trans[0]-((Width/2)*.05)
    point.y = trans[1]-((Height/2)*.05)
    point.z = 0

    rotation = Quaternion()
    rotation.x=rot[0]
    rotation.y=rot[1]
    rotation.z=rot[2]
    rotation.w=rot[3]

    Origin.position = point
    Origin.orientation = rotation

    metaData = MapMetaData()
    metaData.map_load_time = time
    metaData.width = Width 
    metaData.height = Height
    metaData.resolution = Resolution
    metaData.origin = Origin 

    Map = OccupancyGrid()
    Map.info = metaData
    mapData = [0.0]*(Width*Height)

    
    pub_map = rospy.Publisher("/map", OccupancyGrid)
    
    rospy.Subscriber("/scan", LaserScan, callback_laser)
    rospy.loginfo("init")
    rospy.spin()

