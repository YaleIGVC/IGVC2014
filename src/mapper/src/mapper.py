#!/usr/bin/env python
import math
from operator import add
import rospy
import numpy
from rospy.exceptions import ROSException, ROSInterruptException
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
from frame_grabber_node.msg import ImageWithTransform
import cv2
from cv_bridge import CvBridge, CvBridgeError

Resolution = .05 #meters per cell
image_resolution = .1
Width = 2000 #100 meters * 20 cells per meter
Height = 2000 #100 meters * 20 cells per meter
MinLaserRange = .2 #minimum distance for obstacles to be considered

def callback_laser(msg_in):
    global pub_map
    global pub_map_metadata
    global metaData
    global tf_listener
    global Origin
    global origin_angles
    global Map
    global mapData

    angle_min = msg_in.angle_min
    angle_max = msg_in.angle_max
    angle_increment = msg_in.angle_increment
    max_range = msg_in.range_max
    laser_ranges = msg_in.ranges

    if(not initialized):
        Origin = msg_in.info.origin
        origin_quat = [Origin.orientation.x,
                      Origin.orientation.y,
                      Origin.orientation.z,
                      Origin.orientation.w]
        origin_angles = euler_from_quaternion(origin_quat)
        rospy.Subscriber("/lanes_and_flags", ImageWithTransform, callback_image_map, queue_size=1, buff_size = 2**30)

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
    metaData.map_load_time = rospy.get_rostime()
    Map.data = mapData

    rospy.loginfo("Publishing a map")

    pub_map.publish(Map)
    pub_map_metadata.publish(metaData)

def callback_image(msg_in):
    global mapData
    
    print "here"
    
    image_tf = msg_in.tf

    bridge = CvBridge()

    try:
        image_data = bridge.imgmsg_to_cv2(msg_in.image, "bgr8")
    except CvBridgeError as e:
        print e, ": Ros Image to Numpy error"
 
    (image_width, image_height, _temp_) = image_data.shape
    
    resize_width = 50
    resize_height = resize_width * (image_height/image_width)
    
    image_data = cv2.resize(image_data, (resize_width, resize_height))
    
    (image_width, image_height, _temp_) = image_data.shape
    for x in range (0, image_width):
        for y in range(0, image_height):
            if max(image_data[x][y]) == 255:
                image_x = ((x-(image_width/2))*image_resolution)
                image_y = ((y-(image_height/2))*image_resolution)

                r = math.sqrt(math.pow(image_x, 2) + math.pow(image_y, 2))
                image_theta = math.atan2(image_y, image_x)

                image_quat = [image_tf.rotation.x,
                          image_tf.rotation.y,
                          image_tf.rotation.z,
                          image_tf.rotation.w]
                image_angles = euler_from_quaternion(image_quat)

                map_theta = origin_angles[2] - image_angles[2]

                mapx = image_tf.translation.x - Origin.position.x
                mapx = mapx + (r*math.cos(image_theta+map_theta))
                mapy = image_tf.translation.y - Origin.position.y
                mapy = mapy + (r*math.sin(image_theta+map_theta))

                x_cell = int(round(mapx*(1/Resolution)))
                y_cell = int(round(mapy*(1/Resolution)))
               
                if x_cell<0 or x_cell>(Width-1) or y_cell<0 or y_cell>(Height-1):
                    print "Outside map bounds!!!!"

                else:
                    index = ((y_cell*Width)+x_cell)
                    if mapData[index] < 100:
                        mapData[index] = 100

if __name__=='__main__':
    global pub_map
    global pub_map_metadata
    global metaData
    global tf_listener
    global Origin
    global Map
    global mapData
    global initialized

    rospy.init_node('mapper')
    time = rospy.get_rostime()

    tf_listener = tf.TransformListener()

    initialized = False
    
    Origin = Pose()
    try:
         tf_listener.waitForTransform("odom_combined", "base_link", rospy.Time(0), rospy.Duration(3.0))
         (trans,rot) = tf_listener.lookupTransform("odom_combined", "base_link", rospy.Time(0))
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

    # draw initial starting boundary
    #startingBoundaryBoolean = rospy.get_param("starting_line_obstacle", False)
    #startLineLength = 6.5/Resolution
    #startWidth = (Width/2) - (startLineLength/2)
    #startHeight = (Height/2) - (1.5/Resolution)

    #for i in range(int(startWidth), int(startWidth) + int(startLineLength)):
        #mapData[((int(startHeight)*Width)+i)] = 100
    #
    #for i in range(int(startWidth), int(startWidth) + int(startLineLength)):
    #    mapData[((int(startHeight)*Width)+i)] = 100


    
    pub_map = rospy.Publisher("/map", OccupancyGrid)
    pub_map_metadata = rospy.Publisher("/map_metadata", MapMetaData)
    
    rospy.Subscriber("/scan", LaserScan, callback_laser, queue_size=1, buff_size = 2**24)
    rospy.loginfo("init")
    #while(True):
    #    try:
    #        callback_laser(rospy.wait_for_message("/scan", LaserScan, 1)) 
    #    except (ROSException, ROSInterruptException) as e:
    #        print "Mapper not recieving laser scans"
    rospy.spin()
