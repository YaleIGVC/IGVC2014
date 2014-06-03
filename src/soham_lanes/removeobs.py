import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('ittoo.png')

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# define range of required pixels in HSV
lower_d = np.array([0,0,0])
upper_d = np.array([255,255,70])

# Thresholding
darkmask = cv2.inRange(hsv, lower_d, upper_d)
dst = cv2.bitwise_and(img,img,mask = darkmask)

cv2.imshow('roadrunner',dst)
cv2.waitKey(0)
cv2.destroyAllWindows()