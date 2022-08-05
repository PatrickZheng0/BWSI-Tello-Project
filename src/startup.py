#!/usr/bin/python3
 
from tracemalloc import start
import rospy
from just_drone.msg import Dimensions
import time


class Startup:
    def __init__(self):
        rospy.init_node('startup', anonymous=True)
        self.start_publisher = rospy.Publisher('tello/start', Dimensions,
                                               queue_size=1)
        
    def publish_start(self, w, h, d):
        start_msg = Dimensions()
        start_msg.width = w
        start_msg.height = h
        start_msg.distance = d
        self.start_publisher.publish(start_msg)


if __name__ == '__main__':
    try:
        time.sleep(1)
        print("\nStartup node running...\n")
        startup = Startup()
        while True:
            try:
                tv_width = float(input("\nInput TV width in cm: "))
                break
            except ValueError:
                print("Error: Please input a number.")

        while True:    
            try:
                tv_height = float(input("\nInput TV height in cm: "))
                break
            except ValueError:
                print("Error: Please input a number.")
        
        while True:    
            try:
                tv_distance = float(input("\nInput desired distance from TV in cm: "))
                if tv_distance < 50:
                    print("Error: Distance must be at least 50 cm.")
                else:
                    break
            except ValueError:
                print("Error: Please input a number.")

        sound = True # delete when adding sound takeoff
        while not rospy.is_shutdown():
            if sound: # check for start sound
                startup.publish_start(tv_width, tv_height, tv_distance)
                break
        print("\nStartup node finished\n")
    except rospy.ROSInterruptException:
        pass
