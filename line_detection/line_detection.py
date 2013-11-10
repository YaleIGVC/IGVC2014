#! /usr/bin/env python

import cv2, math
import numpy as np
import sys

### Constants
Hlow = 0
Hhigh = 180
Slow = 0
Shigh = 125
Vlow = 50
Vhigh = 255

Offset = 0

def max_min_box(box):
        # returns [minx maxx miny maxy]
        [xs,ys] = map(list, zip(*box))
        minx = min(xs)
        maxx = max(xs)
        miny = min(ys)
        maxy = max(ys)
        return [minx, maxx, miny, maxy]

def replace_contour(largest_contour, img):
        ## Find the box excompassing the largest red blob
        moment = cv2.moments(largest_contour)
        print moment["m00"]
        if moment["m00"] > 150:

            pheight, pwidth, pdepth = img.shape

            rect = cv2.minAreaRect(largest_contour)
            box = cv2.cv.BoxPoints(rect)
            box = np.int0(box)
            # if (not_min_size(cv2.contourArea(largest_contour))): # minimum threshold
            #         return img
            # print box
            print largest_contour
            [minx,maxx,miny,maxy] = max_min_box(box)

            box_width = maxx-minx
            box_height = maxy-miny

            ## Replace pixels in blob
            for xy in largest_contour:
            	x = xy[0][0]
            	y = xy[0][1]
            	img[y][x] = [0, 0, 0]
            # for x in range (minx, maxx):
            #          for y in range (miny, maxy):
            #                  # if y > 0 and y < s and x > 0 and x < w :
            #                          # print img.shape, y, x
            #                 px = img[y][x]
            #                 if (px[0] >= Hlow and px[1] >= Slow and px[2] >= Vlow and px[0] <= Hhigh and px[1] <= Shigh and px[2] <= Vhigh): 
            #                         img[y][x] = [0, 0, 0]
        return img

def draw_box(largest_contour, img):
        ## Find the box excompassing the largest red blob
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        box = np.int0(box)
        cv2.drawContours(img,[box], 0, (0, 0, 255), 2)
        return img

################################

if len(sys.argv) != 2:
        print "Usage: drunkreplace filename"
        sys.exit()

# Get image
orig_img = cv2.imread(sys.argv[1])
height, width, depth = orig_img.shape

# Downsample image
while height > 1000 or width > 1000:
        orig_img = cv2.resize(orig_img, (int(width*.8) , int(height*.8)))
        height, width, depth = orig_img.shape

# Preprocess, convert to HSV
img = cv2.cvtColor(orig_img, cv2.COLOR_BGR2HSV)

# Set the bounds for line selection range
line_lower = np.array([Hlow, Slow, Vlow],np.uint8)
line_upper = np.array([Hhigh, Shigh, Vhigh],np.uint8)
line_binary = cv2.inRange(img, line_lower, line_upper)

dilation = np.ones((int(width/100), int(width/100)), "uint8")

# Red binary is the set of red blobs
line_binary = cv2.dilate(line_binary, dilation)
cv2.imwrite('line_binary.png',line_binary)

# Find the sets of red blobs
contours, hierarchy = cv2.findContours(line_binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

# Discard possibly bad contours
## Too small
# for x, c in enumerate(contours): #repetitive loop???
#         if not_min_size(cv2.contourArea(contours[x]), height, width):
#                 del contours[x] #eliminate any irrelevant contours

## Doesn't fit cup dimensions
# for x, c in enumerate(contours): #repetitive loop???

#         if not_min_size(cv2.contourArea(contours[x]), height, width):
#                 del contours[x] #eliminate any irrelevant contours

## Cup doesnt fill box area
#
#

## Too little area in box

if not contours:
        print "no contours"
        sys.exit()

# Sort contours by area
contoursSorted = sorted(contours, key = lambda (v): cv2.contourArea(v) , reverse = True)

# Write contour lines
for x in xrange(0,len(contours)):
        cv2.drawContours(orig_img, contoursSorted, x, [x*10,0,100],thickness = 5)
cv2.imwrite('contours.png',orig_img)

# # Write freature boxes
# a = img
# for x in xrange(0,len(contours)):
#         a = draw_box(contoursSorted[x], a)
# a = cv2.cvtColor(a, cv2.COLOR_HSV2BGR)
# cv2.imwrite('boxes.png',a)

# Replace image in cups
for x in xrange(0,len(contours)):
        img = replace_contour(contoursSorted[x], img)
out_img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
cv2.imwrite('lines.png',out_img)