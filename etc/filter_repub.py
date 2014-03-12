import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

def callback(msg_in):
    a  = msg_in.pose.pose.orientation.x
    i  = msg_in.pose.pose.orientation.y
    j  = msg_in.pose.pose.orientation.z
    k  = msg_in.pose.pose.orientation.w

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

    msg_imu.linear_acceleration.x = msg_in.twist.twist.linear.x
    msg_imu.linear_acceleration.y = msg_in.twist.twist.linear.y
    msg_imu.linear_acceleration.z = msg_in.twist.twist.linear.z
    global pub
    pub.publish(msg_imu)

if __name__=='__main__':
    rospy.init_node('odom_repub',anonymous=True)
    global pub
    pub = rospy.Publisher("/odom_repub", Imu)
    rospy.Subscriber("/segway_rmp_node/odom", Odometry, callback)
    rospy.loginfo("init")
    rospy.spin()
