#!/usr/bin/python3
 
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from just_drone.msg import Dimensions
import time
from cv_bridge import CvBridge, CvBridgeError


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


def get_dist_to_tv(tv_height_pxls, tv_height):
    # focal length = 920 for mm, 92 for cm
    # uses pinhole camera model
    return 92*tv_height_pxls/tv_height


class Tv:
    def __init__(self):
        rospy.init_node('tv', anonymous=True)

        self.start = False
        self.tv_width = 0
        self.tv_height = 0
        self.bridge = CvBridge()

        self.tv_pub = rospy.Publisher('tello/tv_cmd', Twist, queue_size=10)
        self.tv_cam_pub = rospy.Publisher('tello/tv_cam', Image, queue_size=10)
        self.cam_sub = rospy.Subscriber('tello/camera', Image, self.cam_callback)
        self.start_sub = rospy.Subscriber('tello/start', Dimensions, self.start_callback)
        
    def publish_tv_cmd(self):
        tv_msg = Twist(0,0,0,0) # replace with code to get velocity based on tv position
        self.tv_pub.publish(tv_msg)

    def cam_callback(self, data):  
        # Read the image and make a grayscale version of it      
        img = self.bridge.imgmsg_to_cv2(data, encoding='bgr8')
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Make a copy of the image to crop later
        cropped_img = img.copy()

        # Filter out the brighter pixels from the TV and create a contour for it
        ret, img_thresh = cv2.threshold(gray_img, 100, 255, cv2.THRESH_BINARY)
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

        # gets distance from drone to tv
        dist = get_dist_to_tv(h, self.tv_height)
        print(dist)

        try:
            self.tv_cam_pub.publish(
                self.bridge.cv2_to_imgmsg(data, encoding='bgr8')
            )
        except CvBridgeError as e:
            rospy.logerr(e)

    def start_callback(self, data):
        self.tv_width = data.width
        self.tv_height = data.height
        self.start = True


if __name__ == '__main__':
    try:
        tv = Tv()
        while not tv.start:
            pass
        print("TV node running...")

        while not rospy.is_shutdown():
            pass
        
    except rospy.ROSInterruptException:
        pass
