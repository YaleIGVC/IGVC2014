import rospy
import tf
from nav_msgs import Odometry
from sensor_msgs.msg import Imu

class Repub:
    def __init__(self):
        rospy.init_node('odom_repub',anonymous=True)
        self.pub = rospy.Publisher("/odom_repub", Imu)
        rospy.Subscriber("/segway_rmp_node/odom", Odometry, self.callback)

    def callback(self,msg_in):
        a  = msg_in.pose.pose.orientation.x
        i  = msg_in.pose.pose.orientation.y
        j  = msg_in.pose.pose.orientation.z
        k  = msg_in.pose.pose.orientation.w

        q = tf.transformations.euler_from_quaternion(a, i, j,k)

        msg_imu = Imu()
        msg_imu.header.stamp          = msg_in.header.stamp
        msg_imu.header.frame_id       = msg_in.header.frame_id

        # HACK: Use angular_velocity field to represent orientation!!!
        msg_imu.angular_velocity.x    = q[0]
        msg_imu.angular_velocity.y    = q[1]
        msg_imu.angular_velocity.z    = q[2]

        msg_imu.linear_acceleration.x = msg_in.twist.twist.linear.x
        msg_imu.linear_acceleration.y = msg_in.twist.twist.linear.y
        msg_imu.linear_acceleration.z = msg_in.twist.twist.linear.z
        self.pub.publish(msg_imu)

    def go(self):
        rospy.spin()

if __name__=='__main__':
    r = Repub()
    r.go()

