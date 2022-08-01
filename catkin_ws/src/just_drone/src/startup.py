#!/usr/bin/python3
 
import rospy
from std_msgs.msg import Empty


class Startup:
    def __init__(self):
        rospy.init_node('startup', anonymous=True)
 
        self.start_publisher = rospy.Publisher('tello/start', Empty, queue_size=1)
        
    def publish_start(self):
        start_msg = Empty()

        self.start_publisher.publish(start_msg)


if __name__ == '__main__':
    try:
        startup = Startup()

        while not rospy.is_shutdown():
            # check for start sound
            startup.publish_start()
        
    except rospy.ROSInterruptException:
        pass