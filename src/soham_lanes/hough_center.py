import cv2
import numpy as np

img = cv2.imread('/media/inve/UUI/data/fridayxcampus/2014-05-30-16:02:26-grabbed-aw_image.jpg',0)
#img = cv2.resize(img, (img.shape[1] / 2, img.shape[0] / 2))
img = cv2.medianBlur(img,51)
cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)



circles = cv2.HoughCircles(img,cv2.cv.CV_HOUGH_GRADIENT,1,20,
                            param1=50,param2=30,minRadius=150,maxRadius=250)

circles = np.uint16(np.around(circles))
for i in circles[0,:]:
    # draw the outer circle
    cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

    print "circle coordinates: " + str((i[0],i[1]))

cv2.imshow('detected circles',cimg)
cv2.waitKey(0)
cv2.destroyAllWindows()
