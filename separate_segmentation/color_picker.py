import cv2
import numpy as np
from djitellopy import Tello
import pygame

# initialize tello
tello = Tello()
tello.connect()

print(tello.get_battery())
tello.send_rc_control(0, 0, 0, 0)
tello.streamon()

# variable to represent whether or not a screenshot was taken
screenshotted = False

while True:
    # waits for user to press "space", then saves a
    # screenshot to be used for color picking
    frame_read = tello.get_frame_read()
    img = frame_read.frame
    cv2.imshow('img', img)

    # init pygame
    pygame.init()
    pygame.display.set_mode(size=(300,300))
    pygame.display.init()

    # checks to see if user pressed space, and if so, takes a screenshot
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                cv2.imwrite('Separate segmentation/screenshot_img.jpg', img)
                screenshotted = True
                break
    # if a screenshot was taken and saved, breaks out of loop
    if screenshotted:
        break

pygame.quit()
cv2.destroyAllWindows()

# read screenshotted image
image = cv2.imread('Separate segmentation/screenshot_img.jpg')
hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# initialize lower and upper bound variables
hsv_lower = None
hsv_upper = None

cv2.imshow('image', image)

# callback function for when mouse is moved
def color_picker(event, x, y, flags, params):    
    if event == cv2.EVENT_LBUTTONDOWN:
        global hsv_lower
        global hsv_upper
        # sets lower and upper bound to a range based on hsv value of the pixel user clicks
        hsv_lower = np.array((hsv_img[y, x][0] - 7, hsv_img[y, x][0] - 40, hsv_img[y, x][0] - 40))
        hsv_upper = np.array((hsv_img[y, x][0] + 7, hsv_img[y, x][0] + 40, hsv_img[y, x][0] + 40))
        # sets range of h value to be 0-180, and range of s+v values to be 0-255
        for i in range(3):
            if hsv_lower[i] < 0:
                hsv_lower[i] = 0
            if hsv_upper[i] > 255:
                hsv_upper[i] = 255
        if hsv_upper[0] > 180:
            hsv_upper[0] = 180

        print('press any key to continue')
    
# calls the color_picker callback function when mouse is
# moved and applies actions to image in window 'image'
cv2.setMouseCallback('image', color_picker)

cv2.waitKey(0)
cv2.destroyAllWindows()