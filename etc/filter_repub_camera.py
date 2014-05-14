import rospy
import tf
from frame_grabber.msg import ImageWithTransform
from sensor_msgs.msg import Image

def callback(msg_in):
    global pub
    pub.publish(msg_in.image)

if __name__=='__main__':
    rospy.init_node('repub_camera',anonymous=True)
    global pub
    pub = rospy.Publisher("/raw_image", Image)
    rospy.Subscriber("/raw_image_with_tf", ImageWithTransform, callback)
    rospy.loginfo("init")
    rospy.spin()
