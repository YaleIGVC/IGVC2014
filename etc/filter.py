import fileinput
import tf

itera = -1
valArr = [0] * 4
for line in fileinput.input():
    if (line.strip() == 'orientation:'):
        itera = itera + 1
    elif (itera == 4):
        quaternion = (valArr[0], valArr[1], valArr[2], valArr[3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        itera = -1
        valArr = [0] * 4
        print "Roll: %s, Pitch: %s, Yaw: %s" %(roll, pitch, yaw)
    elif (itera > -1):
        valArr[itera] = line.split(": ")[1]
        itera = itera + 1
