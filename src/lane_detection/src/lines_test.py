#!/usr/bin/env python

import cv2
import numpy as np
import pdb
import time

start_time = time.time()

image = cv2.imread('1.jpg')
template = cv2.imread('template_match.png', 0)
# pdb.set_trace()
# image = cv2.resize(image, (0,0), fx=0.5, fy=0.5) 
image = image[199:990, 545:1356]
cv2.imshow('im_bw', image)
# cv2.imshow('skel', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
# quit(0)
# cv2.imshow('original_image', image)

# minColor = np.array([0, 0, 200],np.uint8)
# maxColor = np.array([180, 255, 255],np.uint8)

# hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# frame_threshed = cv2.inRange(hsv_img, minColor, maxColor)
# cv2.imshow('thresed', frame_threshed)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# cv2.imwrite('output2.jpg', frame_threshed)
# exit(0)
# image = cv2.multiply(image,np.array([3.0]))
# cv2.imshow('original_image2', image)
# exit(0)
# pdb.set_trace()
gray_image_blue_channel, g, r = cv2.split(image)
# w, h = template.shape[::-1]
# res = cv2.matchTemplate(gray_image_blue_channel, template, cv2.TM_CCOEFF_NORMED)
# threshold = 0.5
# loc = np.where( res >= threshold)
# for pt in zip(*loc[::-1]):
#     cv2.rectangle(image, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)

# cv2.imshow('match', image)





# r = cv2.multiply(r, np.array([0.0]))
# cv2.imshow('merged', cv2.merge((gray_image_blue_channel, g, r)))
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# blur = cv2.GaussianBlur(gray_image_blue_channel,(11,11),0)
blur = cv2.medianBlur(gray_image_blue_channel, 19)

ret,thresh = cv2.threshold(blur,100,255,cv2.THRESH_TOZERO)
kernel = np.ones((5,5),np.uint8)
# erosion = cv2.erode(thresh, kernel, iterations = 1)
morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
# minLineLength = 100
# maxLineGap = 10
# lines = cv2.HoughLinesP(morph,1,np.pi/180,100,minLineLength,maxLineGap)
# for x1,y1,x2,y2 in lines[0]:
#     cv2.line(image,(x1,y1),(x2,y2),(0,255,0),2)

# cv2.imshow('original_image', image)
# cv2.imshow('original_image', image)
# cv2.imshow('gray_image', gray_image_blue_channel)
# cv2.imshow('blurred', blur)
# cv2.imshow('threshholded', thresh)
# cv2.imshow('eroded', morph)
# cv2.imshow('original_image', image)

# (thresh, im_bw) = cv2.threshold(morph, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

cv2.imshow('im_bw', morph)

thresh = 50
im_bw = cv2.threshold(morph, thresh, 255, cv2.THRESH_BINARY)[1]
dist_transform = cv2.distanceTransform(im_bw,cv2.cv.CV_DIST_L2,5)
# cv2.imwrite('output2.jpg', im_bw)
contours,hierarchy = cv2.findContours(im_bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# pdb.set_trace()
# cv2.imshow('im_bw', im_bw)
# j = 0
# for (x,y), value in np.ndenumerate(dist_transform):
#     if dist_transform[x][y] < 5:
#         j = j+1
# print j

# pdb.set_trace()

# cv2.imshow('binary', im_bw)


# # For skeleton representation
# size = np.size(morph)
# skel = np.zeros(morph.shape,np.uint8)
# done = False
# element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))


# while( not done):
#     eroded = cv2.erode(morph,element)
#     temp = cv2.dilate(eroded,element)
#     temp = cv2.subtract(morph,temp)
#     skel = cv2.bitwise_or(skel,temp)
#     morph = eroded.copy()
 
#     zeros = size - cv2.countNonZero(morph)
#     if zeros==size:
#         done = True


# cv2.imshow('skel', skel)
cv2.waitKey(0)
cv2.destroyAllWindows()

print time.time() - start_time, "seconds"

 
