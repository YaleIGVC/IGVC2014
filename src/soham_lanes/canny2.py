import cv2
import numpy as np

def CannyThreshold(lowThreshold):
    detected_edges = cv2.GaussianBlur(gray,(11,11),0)
    detected_edges = cv2.Canny(detected_edges,lowThreshold,lowThreshold*ratio,apertureSize = kernel_size)
    dst = cv2.bitwise_and(img,img,mask = detected_edges)  # just add some colours to edges from original image.
    cv2.imshow('canny demo',dst)

lowThreshold = 250
max_lowThreshold = 300
ratio = 3
kernel_size = 5

img = cv2.imread('itt2.jpg')
img = cv2.resize(img, (img.shape[1] / 8, img.shape[0] / 8))
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

cv2.namedWindow('canny demo')

cv2.createTrackbar('Min threshold','canny demo',lowThreshold, max_lowThreshold, CannyThreshold)

CannyThreshold(0)  # initialization
if cv2.waitKey(0) == 27:
    cv2.destroyAllWindows()
