import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('itt1.jpg',0)

blur = cv2.GaussianBlur(img,(11,11),0)

edges = cv2.Canny(blur, 300, 300, apertureSize=5)

plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(edges,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])

plt.show()