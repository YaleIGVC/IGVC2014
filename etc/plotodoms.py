import fileinput
import tf
import time
import pylab

itera = -1
valArr = [0] * 4
roll = list()
pitch = list()
yaw = list()

# plot stuff
f = pylab.figure()
f.show()

for line in fileinput.input():
    if (line.strip() == 'orientation:'):
        itera = itera + 1
    elif (itera == 4):
        quaternion = (valArr[0], valArr[1], valArr[2], valArr[3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll.append(euler[0])
        pitch.append(euler[1])
        yaw.append(euler[2])

        pylab.plot(pitch,'r')
        pylab.draw()
        itera = -1
        valArr = [0] * 4
        print "%s %s %s %f" %(roll[-1], pitch[-1], yaw[-1], time.time())
    elif (itera > -1):
        valArr[itera] = line.split(": ")[1]
        itera = itera + 1
