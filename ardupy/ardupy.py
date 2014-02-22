import serial
import time
ser = serial.Serial('/dev/ttyACM0', 9600)
ser.write ("x") 
time.sleep(2)
readstr = ser.readline()
while readstr != '0\r\n' and readstr != '1\r\n':
	readstr = ser.readline()
if readstr == '1\r\n':
	print '1'
else:
	print '0' 
exit()

