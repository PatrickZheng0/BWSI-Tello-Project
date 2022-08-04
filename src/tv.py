#!/usr/bin/python3
 
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from just_drone.msg import Dimensions
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
        
    def publish_tv_cmd(self, lr, fb, ud, yaw):
        tv_msg = Twist() # replace with code to get velocity based on hand position
        tv_msg.linear.x = lr
        tv_msg.linear.y = fb
        tv_msg.linear.z = ud
        tv_msg.angular.z = yaw
        self.tv_pub.publish(tv_msg)

    def cam_callback(self, data):  
        # Read the image and make a grayscale version of it      
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Make a copy of the image to crop later
        cropped_img = img.copy()

        gray = cv2.GaussianBlur(gray, (7,7), cv2.BORDER_DEFAULT)

        # Filter out the brighter pixels from the TV and create a contour for it
        ret, img_thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)

        # Ensure the only contour detected is the TV screen
        # by takng the largest contour
        biggest_contour, idx = get_largest_contour(contours)
        largest_contour = cv2.drawContours(img, contours, idx, (255, 0, 255), 3)
        bounding_img = largest_contour.copy()

        # Draw a bounding box around the contour
        x, y, w, h = cv2.boundingRect(biggest_contour)
        cv2.rectangle(bounding_img, (x, y), (x + w, y + h), (255, 0, 0), 5)

        # Crop the image to get rid of noise for future color segmentation
        crop_ratio = 0.25
        ratio = int(crop_ratio*h)

        cropped_img[:y + ratio, :] = 0
        cropped_img[y + h - ratio:, :] = 0
        cropped_img[:, :x] = 0
        cropped_img[:, x + w:] = 0

        # find rotation vector to turn camera to face tv
        camera_matrix = np.array([[921.170702, 0.000000, 459.904354],
                                [0.000000, 919.018377, 351.238301],
                                [0.000000, 0.000000, 1.000000]])
        distortion = np.array([-0.033458, 0.105152,
                            0.001256, -0.006647, 0.000000])

        # initializes arrays of 3d points of tv corners
        # where (0,0,0) is center of tv screen
        tv_pts_3d = np.array([
                            [-self.tv_width/2, self.tv_height/2, 0],
                            [self.tv_width/2, self.tv_height/2, 0],
                            [self.tv_width/2, -self.tv_height/2, 0],
                            [-self.tv_width/2, -self.tv_height/2, 0]
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
        # obtain distance from translation vector
        dist = tvec[2][0]
        print(dist)


        # tv controls based on if drone is outside bounding rectangle borders

        # drone is too far from the screen, should go forward
        if dist > 205:
            fb_dir = 1 # forward back direction
        # drone is too close to screen, should move back
        elif dist < 195:
            fb_dir = -1
        # drone is in good spot, don't move
        else:
            fb_dir = 0

        # if drone center is too far left of left
        # rectangle wall then move to the right
        if x > 480:
            lr_dir = 1 # left right direction
        # if drone center is too far right of right
        # rectangle wall then move to the left
        elif (x + w) < 480:
            lr_dir = -1
        # drone is in good spot, don't need to follow tv control
        else:
            lr_dir = 0

        # if drone center is too far down of bottom
        # rectangle wall then move up
        if (y + h) < 360:
            ud_dir = 1 # up down direction
        # if drone center is too far up of top
        # rectangle wall then move down
        elif y > 360:
            ud_dir = -1
        # drone is in good spot, don't need to follow tv control
        else:
            ud_dir = 0

        # drone is turned to the right, turn left
        if yaw_error > 3:
            yaw_dir = -1 # yaw direction
        # drone is turned to the left, turn right
        elif yaw_error < -3:
            yaw_dir = 1
        # drone is in good spot, don't move
        else:
            yaw_dir = 0
        
        print(y+h, "y+h")
        print(y, "y")
        print(bounding_img.shape, "shape")

        tv.publish_tv_cmd(lr_dir*10, fb_dir*7, ud_dir*10, yaw_dir*5)
        # tello.send_rc_control(lr_dir*10, fb_dir*7, ud_dir*10, yaw_dir*5)

        try:
            self.tv_cam_pub.publish(
    #            self.bridge.cv2_to_imgmsg(cropped_img, encoding='bgr8')
                self.bridge.cv2_to_imgmsg(bounding_img, encoding='bgr8')
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