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
        print("Startup node running...")
        startup = Startup()
        while True:
            try:
                tv_width = float(input("Input TV width in inches:"))
                break
            except ValueError:
                print("Error: Please input a number.")

        while True:    
            try:
                tv_height = float(input("Input TV height in inches:"))
                break
            except ValueError:
                print("Error: Please input a number.")

        # while True:
        #     i = tv_dim.find("(")
        #     if i >= 0:
        #         tv_dim = tv_dim[i+1:]
        #     i = tv_dim.find(")")
        #     if i >= 0:
        #         tv_dim = tv_dim[:i]
        #     i = tv_dim.find(" ")
        #     while i >= 0:
        #         tv_dim = tv_dim[:i] + tv_dim[i+1:]
        #         i = tv_dim.find(" ")
        #     try:
        #         i = tv_dim.index(",")
        #         w = tv_dim[:i]
        #         h = tv_dim[i+1:]
        #         w = int(w)
        #         h = int(h)
        #         print("Dimensions received.\n")
        #         break
        #     except:
        #         print("Invalid dimensions. Try again.")
        #         tv_dim = input("\nInput TV dimensions in inches (w,h): ")
        
        sound = True # delete when adding sound takeoff
        while not rospy.is_shutdown():
            if sound: # check for start sound
                startup.publish_start(tv_width, tv_height)
                break
        print("Startup node finished")
    except rospy.ROSInterruptException:
        pass
