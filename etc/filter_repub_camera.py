import rospy
import tf
from frame_grabber_node.msg import ImageWithTransform
from sensor_msgs.msg import Image

def callback_unwarp(msg_in):
    global pub_unwarp
    print "Image found!"
    pub_unwarp.publish(msg_in.image)

def callback_obstacle(msg_in):
    global pub_obstacle
    print "Image found!"
    pub_obstacle.publish(msg_in.image)

if __name__=='__main__':
    rospy.init_node('repub_camera',anonymous=True)
    global pub
    pub_unwarp = rospy.Publisher("/image_unwarp/output_video2", Image)
    pub_obstacle = rospy.Publisher("/lanes_and_flags2", Image)
    rospy.Subscriber("/image_unwarp/output_video", ImageWithTransform, callback_unwarp)
    rospy.Subscriber("/lanes_and_flags", ImageWithTransform, callback_obstacle)
    rospy.loginfo("init")
    rospy.spin()
