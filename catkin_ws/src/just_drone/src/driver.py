#!/usr/bin/python3
 
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from djitellopy import Tello
import cv2
from cv_bridge import CvBridge, CvBridgeError # for converting from cv2 to ros image
from just_drone.msg import dimensions
import time


class Driver:
    def __init__(self):
        rospy.init_node('driver', anonymous=True)

        # initialize other variables
        self.bridge = CvBridge()
        self.hand_cmd = (0,0,0,0)
        self.tv_cmd = (0,0,0,0)
        self.start = False

        # initialize publishers and subscribers
        self.cam_publisher = rospy.Publisher('tello/camera', Image, queue_size=10)
        self.start_subscriber = rospy.Subscriber('tello/start', dimensions, self.start_callback)
        self.land_subscriber = rospy.Subscriber('tello/land', Empty, self.land_callback)
        self.hand_subscriber = rospy.Subscriber('tello/hand_cmd', Twist, self.hand_callback)
        self.tv_subscriber = rospy.Subscriber('tello/tv_cmd', Twist, self.tv_callback)
        
        
    def publish_video(self):
        try:
            # get and publish video stream - access through ros
            img = self.getFrame.frame
            img = cv2.resize(img,(930,710))
            self.cam_publisher.publish(self.bridge.cv2_to_imgmsg(img,encoding='bgr8'))

        except CvBridgeError as e:
            rospy.logerr(e)

    def start_callback(self, data):
        self.start = True

    def land_callback(self, data):
        pass

    def hand_callback(self, data):
        pass

    def tv_callback(self, data):
        pass

    def init_tello(self):
        self.tello = Tello()
        self.tello.connect()
        cam_direction = Tello.CAMERA_FORWARD
        self.tello.set_video_direction(cam_direction)
        self.tello.streamon()
        self.getFrame = self.tello.get_frame_read()


if __name__ == '__main__':
    try:
        driver = Driver()
        while not driver.start:
            time.sleep(10)
        print("\nDriver Node Running...\n")

        import pygame # for emergency land
        pygame.init()
        pygame.display.set_mode(size=(300,300))
        pygame.display.init()

        driver.init_tello()

        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        driver.tello.send_rc_control(0,0,0,0)
                        driver.tello.land()
                        space = True
                        break
            if space:
                break

            driver.publish_video()

    except rospy.ROSInterruptException:
        pass