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
image_resolution = 0.036 / 2 # Scale of 2
Width = 2000 #100 meters * 20 cells per meter
Height = 2000 #100 meters * 20 cells per meter
MinLaserRange = .2 #minimum distance for obstacles to be considered

def callback_laser(msg_in):
    global pub_map
    global pub_map_metadata
    global metaData
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

    for r in laser_ranges:
        x = 0
        y = 0
        if (not math.isnan(r)) and (r < max_range) and (r > MinLaserRange):
            x = trans[0] - Origin.position.x
            y = trans[1] - Origin.position.y

            x = x + (r*math.cos(angles[2]+angle_min-origin_angles[2]))
            y = y + (r*math.sin(angles[2]+angle_min-origin_angles[2]))

            x = int(round(x*(1/Resolution)))
            y = int(round(y*(1/Resolution)))

            if x<0 or x>(Width-1) or y<0 or y>(Height-1):
                print "Image Outside map bounds!!!!"

            else:
                index = ((y*Width)+x)
                if mapData[index] < 100:
                    #mapData[index] = mapData[index] + 5
                    mapData[index] = 100
                    #pass

        angle_min = angle_min + angle_increment


def callback_image(msg_in):
    global mapData
    global Map
    global metaData

    image_tf = msg_in.tf

    bridge = CvBridge()

    try:
        image_data = bridge.imgmsg_to_cv2(msg_in.image, "bgr8")
    except CvBridgeError as e:
        print e, ": Ros Image to Numpy error"

    #image_data = cv2.flip(image_data,0)


    (image_height, image_width, _temp_) = image_data.shape

    print "height: ", image_height, "width", image_width
    mapx = image_tf.translation.x - Origin.position.x
    mapy = image_tf.translation.y - Origin.position.y

    image_quat = [image_tf.rotation.x,
                 image_tf.rotation.y,
                 image_tf.rotation.z,
                 image_tf.rotation.w]
    image_angles = euler_from_quaternion(image_quat)

    map_theta = origin_angles[2] - image_angles[2]
    map_theta = map_theta + math.pi/2
    
    #image_data = cv2.flip(image_data,1)

    #offset_sin = math.sin(map_theta)
    #offset_cos = math.cos(map_theta)

    for x in range (0, image_width):
        for y in range(0, image_height):
            if max(image_data[y][x]) != 0:

                image_x = ((x-(center_x))*image_resolution)
                image_y = ((y-(center_y))*image_resolution)

                r = math.sqrt(math.pow(image_x,2) + math.pow(image_y,2))
                image_theta = math.atan2(image_y, image_x)

                transformed_x = mapx + (r*math.cos(-(image_theta+map_theta)))
                transformed_y = mapy + (r*math.sin(-(image_theta+map_theta)))

                #transformed_x = mapx + (image_x*offset_cos) - (image_y*offset_sin)
                #transformed_y = mapy + (image_x*offset_sin) + (image_y*offset_cos)

                x_cell = int(round(transformed_x*(1/Resolution)))
                y_cell = int(round(transformed_y*(1/Resolution)))

                if x_cell<0 or x_cell>(Width-1) or y_cell<0 or y_cell>(Height-1):
                    print "Outside map bounds!!!!"

                else:
                    index = ((y_cell*Width)+x_cell)
                    if mapData[index] < 100:
                        mapData[index] = 100
    
    Map.header.stamp = rospy.get_rostime()
    Map.header.frame_id = 'map'
    Map.info.map_load_time = rospy.get_rostime()
    metaData.map_load_time = rospy.get_rostime()
    Map.data = mapData

    rospy.loginfo("Publishing a map")

    pub_map.publish(Map)
    pub_map_metadata.publish(metaData)



if __name__=='__main__':
    global pub_map
    global pub_map_metadata
    global metaData
    global tf_listener
    global Origin
    global origin_angles
    global Map
    global mapData
    global initialized
    global center_x
    global center_y

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

    center_x = rospy.get_param("/image_unwarp/center_x")/8
    center_y = rospy.get_param("/image_unwarp/center_y")/8

    center_x = 248 * 2
    center_y = 120 * 2

    origin_angles = [0.0, 0.0, angles[2]]
    rospy.Subscriber("/detected_lanes", ImageWithTransform, callback_image, queue_size=1, buff_size = 2**30)

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
    
    #rospy.Subscriber("/scan", LaserScan, callback_laser, queue_size=1, buff_size = 2**24)
    rospy.loginfo("init")
    #while(True):
    #    try:
    #        callback_laser(rospy.wait_for_message("/scan", LaserScan, 1)) 
    #    except (ROSException, ROSInterruptException) as e:
    #        print "Mapper not recieving laser scans"
    rospy.spin()
