#!/usr/bin/python3
 
import rospy
from std_msgs.msg import Empty
from just_drone.msg import Dimensions


class Startup:
    def __init__(self):
        rospy.init_node('startup', anonymous=True)
        self.start_publisher = rospy.Publisher('tello/start', Dimensions, queue_size=1)
        
    def publish_start(self,w,h):
        start_msg = Dimensions()
        start_msg.width = w
        start_msg.height = h
        self.start_publisher.publish(start_msg)


if __name__ == '__main__':
    try:
        print("\nStartup Node Running...\n")
        startup = Startup()
        tv_w = input("\nInput TV width in cm: ")
        while True:
            try:
                w = int(tv_w)
                break
            except:
                print("Invalid dimensions. Try again.")
                tv_w = input("\nInput TV width in cm: ")
        tv_h = input("\nInput TV height in cm: ")
        while True:
            try:
                w = int(tv_h)
                break
            except:
                print("Invalid dimensions. Try again.")
                tv_w = input("\nInput TV height in cm: ")
        startup.publish_start(w,h)
        print("Dimensions received.\n")
        
        sound = True # delete when adding sound takeoff
        while not rospy.is_shutdown():
            if sound: # check for start sound
                startup.publish_start(w,h)
                break
        print("\nStartup Node Finished\n")
    except rospy.ROSInterruptException:
        pass
