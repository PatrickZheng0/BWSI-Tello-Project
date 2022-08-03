from turtle import distance
import cv2
import numpy as np
from zmq import TCP_KEEPALIVE_IDLE

# tv dimensions in cm
tv_width = 15
tv_height = 8.6

# Return the greatest contour and its index from a sequence of contours
def get_largest_contour(contours):
    # If there are no contours, return None
    if len(contours) == 0:
        return None

    greatest_contour_area = float("-inf")
    greatest_contour = None
    index = None

    for i in range(len(contours)):
        # If the contour area is greater than the greatest recorded area,
        # update the greatest recorded area and the index of the contour
        if cv2.contourArea(contours[i]) > greatest_contour_area:
            greatest_contour = contours[i]
            index = i
            greatest_contour_area = cv2.contourArea(contours[i])

    return greatest_contour, index

# Read the image and make a grayscale version of it
# (Will later replace with Tello feed)
img = cv2.imread('Separate segmentation/screenshot_img.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Make a copy of the image to crop later
cropped_img = img.copy()

# Filter out the brighter pixels from the TV and create a contour for it
ret, img_thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)

# Ensure the only contour detected is the TV screen
# by takng the largest contour
biggest_contour, idx = get_largest_contour(contours)
largest_contour = cv2.drawContours(img, contours, idx, (255, 0, 255), 3)

# Draw a bounding box around the contour
x, y, w, h = cv2.boundingRect(biggest_contour)
cv2.rectangle(largest_contour, (x, y), (x + w, y + h), (255, 0, 0), 5)

# Crop the image to get rid of noise for future color segmentation
crop_ratio = 0.25
ratio = int(crop_ratio*h)

cropped_img[:y + ratio, :] = 0
cropped_img[y + h - ratio:, :] = 0
cropped_img[:, :x] = 0
cropped_img[:, x + w:] = 0

# Show the cropped image
#cv2.imshow('cropped', cropped_img)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

# find rotation vector to turn camera to face tv
camera_matrix = np.array([[921.170702, 0.000000, 459.904354],
                          [0.000000, 919.018377, 351.238301],
                          [0.000000, 0.000000, 1.000000]])
distortion = np.array([-0.033458, 0.105152,
                       0.001256, -0.006647, 0.000000])

# initializes arrays of 3d points of tv corners
# where (0,0,0) is center of tv screen
tv_pts_3d = np.array([
                    [-tv_width/2, tv_height/2, 0],
                    [tv_width/2, tv_height/2, 0],
                    [tv_width/2, -tv_height/2, 0],
                    [-tv_width/2, -tv_height/2, 0]
                    ])
# initializes array of 2d points of corners of tv
# where (0,0) is top left of image taken from tello
tv_pts_2d = np.array([
                    [y, x],
                    [y, x+w],
                    [y+h, x+w],
                    [y+h, x]],
                    dtype=np.float32)
ret, rvec, tvec = cv2.solvePnP(tv_pts_3d, tv_pts_2d, camera_matrix, 
                                distortion, flags=cv2.SOLVEPNP_IPPE)

# obtain yaw_error from rotation vector
yaw_error = rvec[2][0]
print(yaw_error)

# obtain distance from translation vector
dist = tvec[2][0]
print(dist)