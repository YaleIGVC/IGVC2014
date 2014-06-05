import rospy
import tf
from frame_grabber_node.msg import ImageWithTransform
from sensor_msgs.msg import Image
def callback_unwarp(msg_in):
    global pub_unwarp
    pub_unwarp.publish(msg_in.image)

def callback_obstacle(msg_in):
    global pub_obstacle
    pub_obstacle.publish(msg_in.image)

def callback_obstacle2(msg_in):
    global pub_obstacle2
    pub_obstacle2.publish(msg_in.image)

def callback_raw(msg_in):
    global pub_raw
    print "asdf"
    pub_raw.publish(msg_in.image)

if __name__=='__main__':
    rospy.init_node('repub_camera',anonymous=True)
    global pub
    global pub_raw
    pub_unwarp = rospy.Publisher("/image_unwarp/output_video2", Image)
    pub_obstacle = rospy.Publisher("/lanes_and_flags2", Image)
    pub_obstacle2 = rospy.Publisher("/detected_lanes2", Image)
    pub_raw = rospy.Publisher("/raw_image", Image)
    rospy.Subscriber("/raw_image_with_tf", ImageWithTransform, callback_raw)
    rospy.Subscriber("/image_unwarp/output_video", ImageWithTransform, callback_unwarp)
    rospy.Subscriber("/lanes_and_flags", ImageWithTransform, callback_obstacle)
    rospy.Subscriber("/detected_lanes", ImageWithTransform, callback_obstacle2)
    rospy.loginfo("init")
    rospy.spin()
