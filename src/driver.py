#!/usr/bin/python3

import sys
import time

import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from djitellopy import Tello
import cv2
from cv_bridge import CvBridge, CvBridgeError # for converting from cv2 to ros image
from just_drone.msg import Dimensions
import pygame # for emergency land


class Driver:
    def __init__(self):
        rospy.init_node('driver', anonymous=True)

        # initialize other variables
        self.bridge = CvBridge()
        self.hand_cmd = [0, 0, 0, 0] # [x, y, z, yaw]
        self.tv_cmd = [0, 0, 0, 0] # [x, y, z, yaw]
        self.start = False
        self.stop = False

        # initialize publishers and subscribers
        self.cam_pub = rospy.Publisher('tello/camera', Image, queue_size=10)
        self.start_sub = rospy.Subscriber('tello/start', Dimensions, self.start_callback)
        self.land_sub = rospy.Subscriber('tello/land', Empty, self.land_callback)
        self.hand_sub = rospy.Subscriber('tello/hand_cmd', Twist, self.hand_callback)
        self.tv_sub = rospy.Subscriber('tello/tv_cmd', Twist, self.tv_callback)
        
        
    def publish_video(self):
        try:
            # get and publish video stream - access through ros
            img = self.get_frame.frame
            self.cam_pub.publish(
                self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            )

        except CvBridgeError as e:
            rospy.logerr(e)

    def start_callback(self, data):
        self.start = True

    def land_callback(self, data):
        self.stop = True

    def hand_callback(self, data):
        # only changes y and z
        self.hand_cmd[0] = data.linear.x
        self.hand_cmd[2] = data.linear.z

    def tv_callback(self, data):
        # y for distance, yaw to be perpendicular, x and z to stay within tv border
        self.tv_cmd[0] = data.linear.x # left-right
        self.tv_cmd[1] = data.linear.y # forward-back
        self.tv_cmd[2] = data.linear.z # up-down
        self.tv_cmd[3] = data.angular.z # yaw

    def init_tello(self):
        self.tello = Tello()
        self.tello.connect()
        self.tello.streamon()
        self.get_frame = self.tello.get_frame_read()


if __name__ == '__main__':
    try:
        driver = Driver()
        while not driver.start:
            pass
        
        print("Driver Node running...")

        pygame.init()
        pygame.display.set_mode(size=(300, 300))

        driver.init_tello()

        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        driver.stop = True
                        break
                if event.type == pygame.QUIT:
                    break
            if driver.stop:
                break
            driver.publish_video()
            if driver.tv_cmd[0] == 0:
                x = driver.hand_cmd[0]
            else:
                x = driver.tv_cmd[0]
            y = driver.tv_cmd[1]
            if driver.tv_cmd[2] == 0:
                z = driver.hand_cmd[2]
            else:
                z = driver.tv_cmd[2]
            yaw = driver.tv_cmd[3]
            driver.tello.send_rc_control(y,x,z,yaw)

        driver.tello.send_rc_control(0, 0, 0, 0)
        driver.tello.land()
        pygame.quit()
        sys.exit()
    except rospy.ROSInterruptException:
        pass
