from djitellopy import Tello
import cv2
import pygame

tello = Tello()
tello.connect()

print(tello.get_battery())
tello.streamon()

screenshotted = False

while True:
    # waits for user to press "s", then saves a
    # screenshot to be used for color picking
    frame_read = tello.get_frame_read()
    img = frame_read.frame
    cv2.imshow('img', img)

    pygame.init()
    pygame.display.set_mode(size=(300,300))
    pygame.display.init()

    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                cv2.imwrite('screenshot_img.jpg', img)
                screenshotted = True
                break

    if screenshotted:
        break


pygame.quit()
cv2.destroyAllWindows()
tello.end()