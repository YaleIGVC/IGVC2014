import cv2
import numpy as np
import sys

pointcount = 0

# mouse callback function
def draw_circle(event,x,y,flags,param):
	if event == cv2.EVENT_LBUTTONDOWN:
		global pointcount
		cv2.circle(img,(x,y),1,(255,255,255),-1)
		print "coords of point " + str(pointcount) + ": x = " + str(x) + "; y = " + str(y)
		pointcount += 1


try:
	img = cv2.imread(sys.argv[1])
except:
	print "Please pass in a valid image file path as the first argument"
	exit(0)
cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_circle)

while(1):
	try:
		cv2.imshow('image',img)
	except:
		print "Unable to process image/invalid image"
		exit(0)
	if cv2.waitKey(20) & 0xFF == 27:
		break
cv2.destroyAllWindows()