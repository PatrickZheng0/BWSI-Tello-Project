import cv2
import numpy as np
from djitellopy import Tello
import time

def get_largest_contour(contours):
    greatest_contour_area = float("-inf")
    greatest_contour = None
    index = None

    if len(contours) == 0:
        return None
    
    for i in range(len(contours)):
        if cv2.contourArea(contours[i]) > greatest_contour_area:
            greatest_contour = contours[i]
            index = i
            greatest_contour_area = cv2.contourArea(contours[i])

    return greatest_contour, index

#img = cv2.imread('C:/Patrick/BWSI/BWSI-Tello-Project/Separate segmentation/anna_tv.png')
img = cv2.imread('./anna_tv.png')
img = cv2.resize(img, (1152, 768), interpolation=cv2.INTER_AREA)
new_img = img.copy()
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

ret, img_thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
biggest_contour, idx = get_largest_contour(contours)
largest_contour = cv2.drawContours(new_img, contours, idx,(255,0,255),3)

x, y, w, h = cv2.boundingRect(contours[idx])
cv2.rectangle(largest_contour,(x,y), (x+w,y+h), (255,0,0), 5)

cv2.imshow('largest contour w/ bounding box', largest_contour)
# print('top left',x,y)
# print('top right',x+w,y)
# print('bottom left',x,y+h)
# print('bottom right',x+w,y+h)

crop_ratio = 0.25
ratio = int(crop_ratio*h)
cropped_img = new_img[y+ratio:y+h-ratio,x:x+w]

cv2.imshow('cropped', cropped_img)

cv2.waitKey(0)
cv2.destroyAllWindows()