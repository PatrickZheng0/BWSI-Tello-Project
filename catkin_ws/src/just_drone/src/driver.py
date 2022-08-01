#!/usr/bin/python3
 
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from djitellopy import Tello
import cv2
from cv_bridge import CvBridge, CvBridgeError


class Driver:
    def __init__(self):
        rospy.init_node('driver', anonymous=True)

        self.tello = Tello()
        self.tello.connect()
        self.tello.streamon()
        self.getFrame = driver.tello.get_frame_read()

        self.bridge = CvBridge()
        self.getFrame = None
        self.hand_cmd = (0,0,0,0)
        self.tv_cmd = (0,0,0,0)
 
        self.cam_publisher = rospy.Publisher('tello/camera', Image, queue_size=10)
        self.start_subscriber = rospy.Subscriber('tello/start', Empty, self.start_callback)
        self.land_subscriber = rospy.Subscriber('tello/land', Empty, self.land_callback)
        self.hand_subscriber = rospy.Subscriber('tello/hand_cmd', Twist, self.hand_callback)
        self.tv_subscriber = rospy.Subscriber('tello/tv_cmd', Twist, self.tv_callback)
        
        
    def publish_video(self):
        try:
            img = self.getFrame.frame
            img = cv2.resize(img,(960,720))
            self.cam_publisher.publish(self.bridge.cv2_to_imgmsg(img,encoding='bgr8'))

        except CvBridgeError as e:
            rospy.logerr(e)

    def start_callback(self, data):
        pass

    def land_callback(self, data):
        pass

    def hand_callback(self, data):
        pass

    def tv_callback(self, data):
        pass


if __name__ == '__main__':
    try:
        driver = Driver()
        cam_direction = Tello.CAMERA_FORWARD
        driver.tello.set_video_direction(cam_direction)

        while not rospy.is_shutdown():
            driver.publish_video()        
  
    except rospy.ROSInterruptException:
        pass