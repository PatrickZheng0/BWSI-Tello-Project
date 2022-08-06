#!/usr/bin/python3
 
import rospy
from just_drone.msg import Dimensions
import time
from djitellopy import Tello
import cv2
from cv_bridge import CvBridge, CvBridgeError # for converting from cv2 to ros image
import numpy as np

class Startup:
    def __init__(self):
        rospy.init_node('startup', anonymous=True)
        self.start_publisher = rospy.Publisher('tello/start', Dimensions,
                                               queue_size=1)
        
    def publish_start(self, w, h, d, hsv_lower, hsv_higher):
        start_msg = Dimensions()
        start_msg.width = w
        start_msg.height = h
        start_msg.distance = d

        start_msg.h_lower = hsv_lower[0]
        start_msg.s_lower = hsv_lower[1]
        start_msg.v_lower = hsv_lower[2]

        start_msg.h_higher = hsv_higher[0]
        start_msg.s_higher = hsv_higher[1]
        start_msg.v_higher = hsv_higher[2]

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

        # color_picker start ----------------------------------------------------
        # pick hsv values of hand to follow
        # initialize tello
        tello = Tello()
        tello.connect()

        print(tello.get_battery())
        tello.send_rc_control(0, 0, 0, 0)
        tello.streamon()

        while True:
            # waits for user to press "s", then saves a
            # screenshot to be used for color picking
            frame_read = tello.get_frame_read()
            img = frame_read.frame
            cv2.imshow('img', img)

            # checks to see if user pressed space, and if so, takes a screenshot
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                cv2.imwrite('screenshot_img.jpg', img)
                break

        cv2.destroyAllWindows()

        # read screenshotted image
        image = cv2.imread('screenshot_img.jpg')
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # initialize lower and upper bound variables
        hsv_lower = None
        hsv_upper = None

        cv2.imshow('image', image)

        # callback function for when mouse is moved
        def color_picker(event, x, y, flags, params):    
            if event == cv2.EVENT_LBUTTONDOWN:
                img_copy = image.copy()
                global hsv_lower
                global hsv_upper
                # sets lower and upper bound to a range based on hsv value of the pixel user clicks
                hsv_lower = np.array((hsv_img[y, x][0] - 3, hsv_img[y, x][1] - 30, hsv_img[y, x][2] - 30))
                hsv_upper = np.array((hsv_img[y, x][0] + 3, hsv_img[y, x][1] + 30, hsv_img[y, x][2] + 30))
                # sets range of h value to be 0-180, and range of s+v values to be 0-255
                for i in range(3):
                    if hsv_lower[i] < 0:
                        hsv_lower[i] = 0
                    if hsv_upper[i] > 255:
                        hsv_upper[i] = 255
                if hsv_upper[0] > 180:
                    hsv_upper[0] = 180

                mask = np.empty(image.shape)
                mask = cv2.inRange(hsv_img, hsv_lower, hsv_upper, mask)
                masked_img = cv2.bitwise_and(image, image, mask=mask)

                cv2.rectangle(masked_img, (x-20, y-20), (x + 20, y + 20), (255, 0, 0), 5)
                cv2.imshow('masked_img', masked_img)

                font = cv2.FONT_HERSHEY_PLAIN
                text = 'Color Picked! Press any key to continue'
                
                # display confirmation of color picked
                cv2.putText(img_copy, text, (x, y), 
                            font, 1.5, 
                            (0, 0, 0), 
                            7) 
                cv2.putText(img_copy, text, (x, y), 
                    font, 1.5, 
                    (255, 255, 255), 
                    2) 
                cv2.imshow('image', img_copy)

        # calls the color_picker callback function when mouse is
        # moved and applies actions to image in window 'image'
        cv2.setMouseCallback('image', color_picker)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        tello.end()
        # color_picker end ----------------------------------------------------

        sound = True # delete when adding sound takeoff
        while not rospy.is_shutdown():
            if sound: # check for start sound
                startup.publish_start(tv_width, tv_height, tv_distance, hsv_lower, hsv_upper)
                break
        print("\nStartup node finished\n")
    except rospy.ROSInterruptException:
        pass