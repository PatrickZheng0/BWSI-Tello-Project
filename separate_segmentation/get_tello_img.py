from djitellopy import Tello
import cv2
import pygame

tello = Tello()
tello.connect()

print(tello.get_battery())
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
                cv2.imwrite('separate_segmentation/screenshot_img.jpg', img)
                screenshotted = True
                break
    # if a screenshot was taken and saved, breaks out of loop
    if screenshotted:
        break


pygame.quit()
cv2.destroyAllWindows()
tello.end()