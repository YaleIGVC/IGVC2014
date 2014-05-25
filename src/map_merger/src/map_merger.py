#!/usr/bin/env python
import math
import rospy
import tf
from tf.transformations import euler_from_quaternion
import tf.msg
import numpy
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from frame_grabber_node.msg import ImageWithTransform
from vision_control.msg import detectedvision
from cv_bridge import CvBridge, CvBridgeError

image_resolution = 0.0078

def callback_laser_map(msg_in):
    global pub_merged_map
    global image_map
    global Resolution
    global Height
    global Width
    global Origin
    global origin_angles
    global initialized

    if(not initialized):
        initialized = True
        Width = msg_in.info.width
        Height = msg_in.info.height
        Resolution = msg_in.info.resolution
        Origin = msg_in.info.origin
        origin_quat = [Origin.orientation.x,
                      Origin.orientation.y,
                      Origin.orientation.z,
                      Origin.orientation.w]
        origin_angles = euler_from_quaternion(origin_quat)
        image_map = [0]*(Width*Height)
        rospy.Subscriber("/lanes_and_flags", ImageWithTransform, callback_image_map, queue_size=1, buff_size = 2**30)


    combined_map = OccupancyGrid()
    combined_map.info = msg_in.info
    combined_map.header = msg_in.header   
    
    combined_map.data = numpy.maximum(msg_in.data, image_map)
    #combined_map.data = image_map

    pub_merged_map.publish(combined_map)
    rospy.loginfo("Publishing combined_map")

def callback_image_map(msg_in):
    global image_map

    image_tf = msg_in.tf

    bridge = CvBridge()

    try:
        image_data = bridge.imgmsg_to_cv2(msg_in.image, "bgr8")
    except CvBridgeError as e:
        print e, ": Ros Image to Numpy error"
 
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
                    if image_map[index] < 100:
                        image_map[index] = 100

if __name__=='__main__':
    global pub_merged_map
    global image_map
    global initialized

    rospy.init_node('map_merger')

    pub_merged_map = rospy.Publisher("/merged_map", OccupancyGrid)

    rospy.Subscriber("/map", OccupancyGrid, callback_laser_map, queue_size=1, buff_size = 2**30)

    initialized = False

    rospy.loginfo("init")
    rospy.spin()
