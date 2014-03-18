#!/usr/bin/env python
import rospy import tf
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

def callback_imu(msg_in):
    a  = msg_in.orientation.x
    i  = msg_in.orientation.y
    j  = msg_in.orientation.z
    k  = msg_in.orientation.w

    #quaternion = (a,i,j,k)
    #q = tf.transformations.euler_from_quaternion(quaternion)

    msg_out = Odometry()
    msg_out.header.stamp          = msg_in.header.stamp
    msg_out.header.frame_id       = 'odom_frame'

    global x
    global y
    global x_vel
    global y_vel
    global twist_yaw
    msg_out.pose.pose.position.x = x
    msg_out.pose.pose.position.y = y
    msg_out.pose.pose.position.z = 0.0

    msg_out.pose.pose.orientation.x = a
    msg_out.pose.pose.orientation.y = i
    msg_out.pose.pose.orientation.z = j
    msg_out.pose.pose.orientation.w = k

    msg_out.twist.twist.linear.x = x_vel
    msg_out.twist.twist.linear.y = y_vel
    msg_out.twist.twist.linear.z = 0.0
    msg_out.twist.twist.angular.x = 0.0
    msg_out.twist.twist.angular.y = 0.0
    msg_out.twist.twist.angular.z = twist_yaw


    global covariance
    msg_out.pose.covariance = covariance
    msg_out.twist.covariance = [0.0] * 36

    global pub
    pub.publish(msg_out)

def callback_odom(msg_in):
    global x
    global y
    global covariance
    global x_vel
    global y_vel
    global twist_yaw
    x = msg_in.pose.pose.position.x
    y = msg_in.pose.pose.position.y
    x_vel = msg_in.twist.twist.linear.x
    y_vel = msg_in.twist.twist.linear.y
    twist_yaw = msg_in.twist.twist.angular.z
    covariance = msg_in.pose.covariance

if __name__=='__main__':
    rospy.init_node('odom_combined',anonymous=True)
    global pub
    global x
    global y
    global x_vel
    global y_vel
    global twist_yaw

    x = y = x_vel = y_vel = twist_yaw = 0.0

    global covariance
    global yaw

    pub = rospy.Publisher("/odom_combined", Odometry)
    rospy.Subscriber("/imu_data", Imu, callback_imu)
    rospy.Subscriber("/odom", Odometry, callback_odom)
    rospy.loginfo("init")
    rospy.spin()
