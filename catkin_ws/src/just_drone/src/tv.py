#!/usr/bin/python3
 
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from just_drone.msg import dimensions
import time


class TV:
    def __init__(self):
        rospy.init_node('tv', anonymous=True)

        self.start = False
        self.tv_width = 0
        self.tv_height = 0

        self.tv_publisher = rospy.Publisher('tello/tv_cmd', Twist, queue_size=10)
        self.cam_subscriber = rospy.Subscriber('tello/camera', Image, self.cam_callback)
        self.start_subscriber = rospy.Subscriber('tello/start', dimensions, self.start_callback)
        
    def publish_tv_cmd(self):
        tv_msg = Twist(0,0,0,0) # replace with code to get velocity based on tv position
        self.tv_publisher.publish(tv_msg)

    def cam_callback(self, data):
        pass

    def start_callback(self, data):
        self.tv_width = data.width
        self.tv_height = data.height
        self.start = True


if __name__ == '__main__':
    try:
        tv = TV()
        while not tv.start:
            time.sleep(10)
        time.sleep(2)
        print("\nTV Node Running...\n")

        while not rospy.is_shutdown():
            pass
        
    except rospy.ROSInterruptException:
        pass