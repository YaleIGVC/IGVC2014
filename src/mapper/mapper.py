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

Resolution = .05 #meters per cell
Width = 2000 #100 meters * 20 cells per meter
Height = 2000 #100 meters * 20 cells per meter

def callback_laser(msg_in):
    global pub_map
    global tf_listener
    global tf_broadcaster
    global Origin
    global Map
    global mapData

    tf_broadcaster.sendTransform((0,0,0),(0,0,0,0),rospy.get_rostime(), 'map', 'odom_combined')

    angle_min = msg_in.angle_min
    angle_max = msg_in.angle_max
    angle_increment = msg_in.angle_increment
    ranges = msg_in.ranges

    try:
        (trans,rot) = listener.lookupTransform('odom_combined', 'laser', rospy.get_rostime())
    except (tf.LookupException, tf.ConnectivityException):
        continue
    angles = euler_from_quaternion(rot)


    for r in ranges:
        x = trans[0] - Origin.position.x
        y = trans[1] - Origin.position.y
        x = x + (r/cos(angles[2]+angle_min))
        y = y + (r/sin(angles[2]+angle_min))

        if x<0 or x>Width or y<0 or y>Height:

        else:
            index = index2dTo1d(x,y)
            if mapData[index] < 100
                mapData[index] = mapData[index] + 5

        angle_min = angle_min + angle_increment 

    Map.header.stamp = msg_in.header.stamp
    Map.header.frame_id = 'map'
    Map.data = mapData

    pub_map.publish(Map)

if __name__=='__main__':
    global pub_map
    global tf_listener
    global tf_broadcaster
    global Origin
    global Map
    global mapData

    time = rospy.get_rostime()

    Origin = Pose()
    try:
        (trans,rot) = listener.lookupTransform('odom_combined', 'laser', rospy.get_rostime())
    except (tf.LookupException, tf.ConnectivityException):
        continue
    point = Point()
    point.x = trans[0]-(Width/2)
    point.y = trans[1]-(Height/2)
    point.z = trans[2]

    Origin.position = point
    Origin.orientation = rot

    metaData = MapMetaData()
    metaData.map_load_time = time
    metaData.width = Width 
    metaData.height = Height
    metaData.resolution = Resolution
    metaData.origin = Origin 

    Map = OccupancyGrid()
    Map.info = metaData
    mapData = [0.0]*(Width*Height)

    rospy.init_node('tf_listen')
    tf_broadcaster = tf.TransformBroadcaster()
    tf_listener = tf.TransformListener()
    
    pub_map = rospy.Publisher("/map", OccupancyGrid)
    
    rospy.init_node('mapper',anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback_laser)
    rospy.loginfo("init")
    rospy.spin()

def index2dTo1d(x, y):
    return ((y*Width)+x)