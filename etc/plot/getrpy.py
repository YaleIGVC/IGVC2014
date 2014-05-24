import fileinput
import rospy
import tf
from vectornav.msg import ins

class Plotter:
    def callback(self,data):
        print data.RPY.z

    def listener(self):
        rospy.init_node('listener',anonymous=True)
        rospy.Subscriber("vectornav/ins",ins, self.callback)
        rospy.spin()

if __name__=='__main__':
    p = Plotter()
    p.listener()
