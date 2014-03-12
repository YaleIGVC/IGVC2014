import fileinput
import rospy
import tf
import time
import pylab
from vectornav.msg import ins



def callback(data):
    print "Herehere"
    print "REC'D: ", data.RPY.z

def listener():
    print("Here!")
    rospy.init_node('listener',anonymous=True)
    rospy.Subscriber("vectornav/ins",ins, callback)
    rospy.spin()

if __name__=='__main__':
    #itera = -1
    #valArr = [0] * 3
    #roll = list()
    #pitch = list()
    #yaw = list()
    # plot stuff
    #f = pylab.figure()
    #f.show()

    listener()

#if (line.strip() == 'RPY:'):
    #itera = itera + 1
#elif (itera == 3):
    #roll.append(valArr[0])
    #pitch.append(valArr[1])
    #yaw.append(valArr[2])

    #pylab.plot(yaw,'r')
    #pylab.draw()
    #itera = -1
    #valArr = [0] * 4
    #print "%s %s %s %f" %(roll[-1], pitch[-1], yaw[-1], time.time())
#elif (itera > -1):
    #valArr[itera] = line.split(": ")[1]
    #itera = itera + 1
