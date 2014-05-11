#!/usr/bin/env python
import math
import rospy
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import tf.msg
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from frame_grabber_node.msg import ImageWithTransform

image_resolution = 0 #??????????!!!!!!!!?!?!?!?!?!?!?!?!?!?!

def callback_laser_map(msg_in):
    global pub_merged_map
    global image_map
    global Resolution
    global Height
    global Width
    global initialized
    global laser_map_metadata

    if(not initialized):
        rospy.Subscriber("/detected", ImageWithTransform, callback_image_map)

    header = msg_in.header
    laser_map = msg_in.data
    laser_map_metadata = msg_in.info

def callback_image_map(msg_in):
    global image_map

    image_height = msg_in.image.height
    image_width = msg_in.image.width
    image_data = msg_in.image.data
    image_tf = msg_in.tf

    for x in range (0, image_width):
        for y in range(0, image_height):
            index = ((y*width)+x)
            if image_data[index] = 0: #How are image values stored in ros imag??????????
                x_temp = ((x-(image_width/2))*image_resolution)
                y_temp = ((y-(image_width/2))*image_resolution)

                y = math.sqrt(math.pow(x_temp, 2) + math.pow(y_temp, 2))


                mapx = image_tf.translation.x - laser_map_metadata.origin.position.x
        
                mapy = image_tf.translation.y - laser_map_metadata.origin.position.y






if __name__=='__main__':
    global pub_merged_map
    global image_map
    global initialized

    rospy.init_node('map_merger')

    pub_merged_map = rospy.Publisher("/merged_map", OccupancyGrid)

    rospy.Subscriber("/map", OccupancyGrid, callback_laser_map)
    #GET NAME OF NODE rospy.Subscriber("/IMAGEWITHFLAGDATA!!!", ImageWithTransform, callback_image_map)

    initialized = false

    image_map = [0]*(Width*Height)

    rospy.loginfo("init")
    rospy.spin()