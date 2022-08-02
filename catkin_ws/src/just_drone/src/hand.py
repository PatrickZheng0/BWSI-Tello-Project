#!/usr/bin/python3
 
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import time # for automatic landing timeout
from just_drone.msg import dimensions


class Hand:
    def __init__(self):
        rospy.init_node('hand', anonymous=True)

        self.start = False

        self.hand_publisher = rospy.Publisher('tello/hand_cmd', Twist, queue_size=10)
        self.cam_subscriber = rospy.Subscriber('tello/camera', Image, self.cam_callback)
        self.start_subscriber = rospy.Subscriber('tello/start', dimensions, self.start_callback)
        
    def publish_hand_cmd(self):
        hand_msg = Twist(0,0,0,0) # replace with code to get velocity based on hand position
        self.hand_publisher.publish(hand_msg)

    def cam_callback(self, data):
        pass

    def start_callback(self, data):
        self.start = True


if __name__ == '__main__':
    try:
        hand = Hand()
        while not hand.start:
            time.sleep(10)
        time.sleep(1)
        print("\nHand Node Running...\n")

        while not rospy.is_shutdown():
            pass
        
    except rospy.ROSInterruptException:
        pass