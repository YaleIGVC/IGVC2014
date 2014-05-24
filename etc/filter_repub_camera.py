import rospy
import tf
from frame_grabber_node.msg import ImageWithTransform
from sensor_msgs.msg import Image

def callback(msg_in):
    global pub
    print "Image found!"
    pub.publish(msg_in.image)

if __name__=='__main__':
    rospy.init_node('repub_camera',anonymous=True)
    global pub
    pub = rospy.Publisher("/image_unwarp/output_video2", Image)
    rospy.Subscriber("/image_unwarp/output_video", ImageWithTransform, callback)
    rospy.loginfo("init")
    rospy.spin()
