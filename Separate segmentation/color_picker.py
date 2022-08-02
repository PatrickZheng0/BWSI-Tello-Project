import cv2
import numpy as np

img = cv2.imread('Separate segmentation/anna_tv.png')
img = cv2.resize(img, (1152, 768), interpolation=cv2.INTER_AREA)
hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
hsv_lower = None
hsv_upper = None

cv2.imshow('image', img)

def color_picker(event, x, y, flags, params):    
    if event == cv2.EVENT_LBUTTONDOWN:
        global hsv_lower
        global hsv_upper
        hsv_lower = np.array((hsv_img[y, x][0] - 7, hsv_img[y, x][0] - 40, hsv_img[y, x][0] - 40))
        hsv_upper = np.array((hsv_img[y, x][0] + 7, hsv_img[y, x][0] + 40, hsv_img[y, x][0] + 40))
        for i in range(3):
            if hsv_lower[i] < 0:
                hsv_lower[i] = 0
            if hsv_upper[i] > 255:
                hsv_upper[i] = 255
        if hsv_upper[0] > 180:
            hsv_upper[0] = 180

        print(x,y)
        print(hsv_lower)
        print(hsv_upper)
    

cv2.setMouseCallback('image', color_picker)

cv2.waitKey(0)
cv2.destroyAllWindows()