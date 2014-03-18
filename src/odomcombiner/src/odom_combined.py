#!/usr/bin/env python
import rospy
import tf
import tf.msg
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

#FIXME USE OOP INSTEAD OF GLOBAL VARIABLES!

def callback_imu(msg_in):
    a  = msg_in.orientation.x
    i  = msg_in.orientation.y
    j  = msg_in.orientation.z
    k  = msg_in.orientation.w

    msg_out = Odometry()
    msg_out.header.stamp          = msg_in.header.stamp
    msg_out.header.frame_id       = 'odom_combined'
    msg_out.child_frame_id       = 'base_footprint'

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

    global pub_tf
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = msg_out.header.frame_id
    t.header.stamp = rospy.Time.now()

    t.frame_id = "odom_combined"
    t.child_frame_id = "base_footprint"
    t.transform.translation.x = y # GLOBAL! From callback_odom
    t.transform.translation.y = x # GLOBAL! From callback_odom
    t.transform.translation.z = 0.0

    # Vars from above. From imu_data
    t.transform.rotation.x = a
    t.transform.rotation.y = i
    t.transform.rotation.z = j
    t.transform.rotation.w = k

    tfm = tf.msg.tfMessage([t])
    pub_tf.publish(tfm)

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
    global pub_tf
    global x
    global y
    global x_vel
    global y_vel
    global twist_yaw

    x = y = x_vel = y_vel = twist_yaw = 0.0

    global covariance
    covariance = [0] * 36
    global yaw

    pub = rospy.Publisher("/odom_combined", Odometry)
    pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage)

    rospy.Subscriber("/imu_data", Imu, callback_imu)
    rospy.Subscriber("/odom", Odometry, callback_odom)
    rospy.loginfo("init")
    rospy.spin()
