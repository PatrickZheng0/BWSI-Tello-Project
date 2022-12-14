import cv2
import numpy as np
from djitellopy import Tello
import time


class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0, current_time=None):
        # set K values
        self.Kp = P
        self.Ki = I
        self.Kd = D
        
        # set time variables
        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()


    def tracker(self, center, current_time=None):
        w = 960 #width of camera feed
        h = 720 #height of camera feed
        
        # initialize tracking variables
        Vx = 0 # x velocity
        Vy = 0 # y velocity
        Ix = 0 # x starting I (increases over time)
        Iy = 0 # y starting I (increases over time)
        prev_errorX = 0 # previous x error
        prev_errorY = 0 # previous y error

        cX = center[0] # center of hand
        cY = center[1] # center of hand

        errorX = cX - (w/2) # x error from center of camera
        errorY = cY - (h/2) # y error from center of camera

        self.current_time = current_time if current_time is not None else time.time()
        delta_t = self.current_time - self.last_time # elapsed time
                
        # control equations                
        if delta_t > self.sample_time:
            # update PID for x
            Px = self.Kp*errorX # proportional - K*error
            Ix += errorX*delta_t # integral - add new area
            Dx = self.Kd*(errorX - prev_errorX)/(delta_t) # derivative - change in x / change in time

            # update PID for y
            Py = self.Kp*errorY
            Iy += errorY*delta_t
            Dy = self.Kd*(errorY - prev_errorY)/(delta_t)
            
            # update velocities
            Vx = Px + (self.Ki * Ix) + Dx
            Vy = Py + (self.Ki * Iy) + Dy

            print(errorX)
            print(errorY)
            if abs(errorX) > 0.5 or abs(errorY) > 0.5: # don't adjust for small errors
                print('vels', Vx, Vy)
                tello.send_rc_control(int(round(Vx)), 0, int(round(Vy)), 0) # round velocities to integers, send x and y velocities to tello
            else: # if small error, send 0 velocities
                tello.send_rc_control(0, 0, 0, 0)

            # update for next iteration
            prev_errorX = errorX
            prev_errorY = errorY
            self.last_time = self.current_time
                    

    def set_sample_time(self, sample_time):
        self.sample_time = sample_time


    def clear(self):
        self.last_error = 0.0


def get_largest_contour(contours):
    # initialize greatest contour variables
    greatest_contour_area = float("-inf")
    greatest_contour = None

    if len(contours) == 0: # no contours
        return None
    
    # find largest contour
    for i in range(len(contours)):
        if cv2.contourArea(contours[i]) > greatest_contour_area:
            greatest_contour = contours[i]
            greatest_contour_area = cv2.contourArea(contours[i])

    return greatest_contour


def get_highest_contour(contours):
    highest_contour_row = float("inf")
    highest_contour = None

    if len(contours) == 0:
        return None

    for contour in contours:
        M = cv2.moments(contour)
        center_row = M["m01"]//M["m00"]
        if center_row < highest_contour_row:
            highest_contour = contour
            highest_contour_row = center_row

    return highest_contour


def find_contours(mask):
    return cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]


def get_contour_center(contour):
    M = cv2.moments(contour) #moments

    if M["m00"] <= 0:
        return None

    #weighted average of moments
    center_col = M["m10"]//M["m00"] 
    center_row = M["m01"]//M["m00"]

    return (center_col, center_row)


if __name__ == "__main__":
    
    fps = 4 # number of images to process per second
    wait_time = 1/fps # time between images

    # hand color range - will change for different videos/dancers
    hsv_lower = np.array((25, 150, 200))
    hsv_upper = np.array((35, 255, 255))

    # initalize PID
    hand_pid = PID(P=0.1, I=0.0, D=0.0)
    hand_pid.set_sample_time(wait_time)

    # for emergency land
    import pygame

    pygame.init()
    pygame.display.set_mode(size=(300,300))
    pygame.display.init()
    space = False

    # initialize tello
    tello = Tello()
    tello.connect()
    print(tello.get_battery())
    tello.send_rc_control(0, 0, 0, 0)
    tello.takeoff()
    tello.streamon()
    frame_read = tello.get_frame_read()
    tello.move_up(50) # height of tv - may change for different tvs

    start = time.time()
    

    while True:
        curr = time.time()
        if curr - start >= wait_time: # only process and image specified number of times per second
            img = frame_read.frame

            hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

            # remove other colors
            mask = np.empty(img.shape)
            mask = cv2.inRange(hsv_img, hsv_lower, hsv_upper, mask)
            # since tello drone gets image in rgb, bitwise_and uses img instead of rgb_img
            masked_img = cv2.bitwise_and(img, img, mask=mask)

            contours = find_contours(mask)
            if contours:
                # find center of largest contour (probably hand)
                largest_contour = get_largest_contour(contours)
                contour_center = get_contour_center(largest_contour)
                
                if contour_center:
                    center = np.array(contour_center)
                    center[1] = img.shape[0] - contour_center[1] # change (0,0) of image from top left to bottom left
                    hand_pid.tracker(center)
            else:
                tello.send_rc_control(0, 0, 0, 0)
        
            cv2.imshow("segmented img", masked_img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            tello.land()
            break
        
        # emergency land
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    tello.send_rc_control(0,0,0,0)
                    tello.land()
                    space = True
                    break
            if event.type == pygame.QUIT:
                pygame.quit()
        if space:
            break

print(f"Battery: {tello.get_battery()}%")
#tello.land()
