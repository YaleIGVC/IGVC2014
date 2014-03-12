import rospy
import tf
from sensor_msgs.msg import Imu

def callback(msg_in):
    a  = msg_in.orientation.x
    i  = msg_in.orientation.y
    j  = msg_in.orientation.z
    k  = msg_in.orientation.w

    quaternion = (a,i,j,k)
    q = tf.transformations.euler_from_quaternion(quaternion)

    msg_imu = Imu()
    msg_imu.header.stamp          = msg_in.header.stamp
    msg_imu.header.frame_id       = msg_in.header.frame_id

    # HACK: Use angular_velocity field to represent orientation!!!
    PI = 3.141592653589
    msg_imu.angular_velocity.x    = (q[0] * 180.0) / PI
    msg_imu.angular_velocity.y    = (q[1] * 180.0)/ PI
    msg_imu.angular_velocity.z    = (q[2] * 180.0) / PI

    msg_imu.linear_acceleration.x = 0
    msg_imu.linear_acceleration.y = 0
    msg_imu.linear_acceleration.z = 0
    global pub
    pub.publish(msg_imu)

if __name__=='__main__':
    rospy.init_node('odom_repub_vectornav',anonymous=True)
    global pub
    pub = rospy.Publisher("/odom_repub_vectornav", Imu)
    rospy.Subscriber("/Imu", Imu, callback)
    rospy.loginfo("init")
    rospy.spin()
