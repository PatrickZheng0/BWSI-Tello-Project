#!/usr/bin/python3
 
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2


class TV:
    def __init__(self):
        rospy.init_node('tv', anonymous=True)

        self.tv_publisher = rospy.Publisher('tello/tv_cmd', Twist, queue_size=10)
        self.cam_subscriber = rospy.Subscriber('tello/camera', Image, self.cam_callback)
        
    def publish_tv_cmd(self):
        tv_msg = Twist(0,0,0,0) # replace with code to get velocity based on tv position
        self.tv_publisher.publish(tv_msg)

    def cam_callback(self, data):
        pass


if __name__ == '__main__':
    try:
        tv = TV()

        while not rospy.is_shutdown():
            pass
        
    except rospy.ROSInterruptException:
        pass