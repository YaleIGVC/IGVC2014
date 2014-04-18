#!/usr/bin/env python
import rospy
import tf
from tf.transformations import euler_from_quaternion
import tf.msg
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData

def callback_laser(msg_in):
    global pub_map
    global pub_map_odom_tf
    global pub_map_metadata
    global time
    global tf_listener
    global tf_broadcaster

    tf_broadcaster.sendTransform((0,0,0),(0,0,0,0),rospy.get_rostime(), "map", "odom_combined")
    angle_min = msg_in.angle_min
    angle_max = msg_in.angle_max
    angle_increment = msg_in.angle_increment
    ranges = msg_in.ranges

    try:
        (trans,rot) = listener.lookupTransform('map', 'laser', rospy.get_rostime())
    except (tf.LookupException, tf.ConnectivityException):
        continue
    angles = euler_from_quaternion(rot)

    angle_min += angles[2]
    angle_max += angles[2]

    msg_out = MapMetaData




if __name__=='__main__':
    global pub_map
    global pub_map_odom_tf
    global pub_map_metadata
    global time
    global tf_listener
    global tf_broadcaster

    time = rospy.get_rostime()

    rospy.init_node('tf_listen')
    tf_broadcaster = tf.TransformBroadcaster()
    tf_listener = tf.TransformListener()
    
    pub_map = rospy.Publisher("/tf", tf.msg.tfMessage)
    pub_map_odom_tf = rospy.Publisher("/map", OccupancyGrid)
    pub_map_metadata = rospy.Publisher("/map_metadata", MapMetaData)
    
    rospy.init_node('mapper',anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback_laser)
    rospy.loginfo("init")
    rospy.spin()