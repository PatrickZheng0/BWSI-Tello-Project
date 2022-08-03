#!/usr/bin/python3
 
import rospy
from std_msgs.msg import Empty
from just_drone.msg import Dimensions


class Startup:
    def __init__(self):
        rospy.init_node('startup', anonymous=True)
        self.start_publisher = rospy.Publisher('tello/start', Dimensions,
                                               queue_size=1)
        
    def publish_start(self, w, h):
        start_msg = Dimensions()
        start_msg.width = w
        start_msg.height = h
        self.start_publisher.publish(start_msg)


if __name__ == '__main__':
    try:
        print("\nStartup node running...\n")
        startup = Startup()
        while True:
            try:
                tv_width = float(input("\nInput TV width in inches: "))
                break
            except ValueError:
                print("Error: Please input a number.")

        while True:    
            try:
                tv_height = float(input("\nInput TV height in inches: "))
                break
            except ValueError:
                print("Error: Please input a number.")
              
        sound = True # delete when adding sound takeoff
        while not rospy.is_shutdown():
            if sound: # check for start sound
                startup.publish_start(tv_width, tv_height)
                break
        print("\nStartup node finished\n")
    except rospy.ROSInterruptException:
        pass
