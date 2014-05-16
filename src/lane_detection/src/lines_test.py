#!/usr/bin/env python

import cv2
import numpy as np
import pdb
import time
from matplotlib import pyplot as plt


start_time = time.time()

image = cv2.imread('t3.bmp')
image = cv2.resize(image, (image.shape[1] / 2, image.shape[0] / 2))
cv2.imshow('image', image)
# image = image[199:990, 545:1356]
image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
# cv2.imshow('image', image)
white_min = np.array([0, 0, 0], np.uint8)
white_max = np.array([62, 255, 255], np.uint8)

frame_threshed = cv2.inRange(image, white_min, white_max)
frame_threshed = 255 - cv2.cvtColor(frame_threshed, cv2.COLOR_GRAY2RGB)
# pdb.set_trace()
new_hsv = np.bitwise_and(frame_threshed, image)
cv2.imshow('frame_threshed', frame_threshed)
cv2.imshow('new', new_hsv)

image = cv2.cvtColor(new_hsv, cv2.COLOR_HSV2BGR)
cv2.imshow('new new image', image)

cv2.waitKey(0)

gray_image_blue_channel, g, r = cv2.split(image)

blur = cv2.medianBlur(gray_image_blue_channel, 11)

ret,thresh = cv2.threshold(blur,100,255,cv2.THRESH_TOZERO)
kernel = np.ones((5,5),np.uint8)
morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

# cv2.imshow('im_bw', morph)

thresh = 50
im_bw = cv2.threshold(morph, thresh, 255, cv2.THRESH_BINARY)[1]
# dist_transform = cv2.distanceTransform(im_bw,cv2.cv.CV_DIST_L2,5)
# contours,hierarchy = cv2.findContours(im_bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# names = ['image','gray_image_blue_channel','blur','thresh','morph','im_bw']


cv2.imshow('image', image)
cv2.imshow('gray_image_blue_channel', gray_image_blue_channel)
cv2.imshow('blur', blur)
cv2.imshow('thresh', thresh)
cv2.imshow('morph', morph)
cv2.imshow('im_bw', im_bw)
cv2.waitKey(0)
cv2.destroyAllWindows()

print time.time() - start_time, "seconds"

 
