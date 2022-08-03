import cv2


# Return the greatest contour and its index from a sequence of contours
def get_largest_contour(contours):
    # If there are no contours, return None
    if len(contours) == 0:
        return None

    greatest_contour_area = float("-inf")
    greatest_contour = None
    index = None

    for i in range(len(contours)):
        # If the contour area is greater than the greatest recorded area,
        # update the greatest recorded area and the index of the contour
        if cv2.contourArea(contours[i]) > greatest_contour_area:
            greatest_contour = contours[i]
            index = i
            greatest_contour_area = cv2.contourArea(contours[i])

    return greatest_contour, index


# Read the image and make a grayscale version of it
# (Will later replace with Tello feed)
img = cv2.imread('Separate segmentation/anna_tv.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Make a copy of the image to crop later
cropped_img = img.copy()

# Filter out the brighter pixels from the TV and create a contour for it
ret, img_thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)

# Ensure the only contour detected is the TV screen
# by takng the largest contour
biggest_contour, idx = get_largest_contour(contours)
largest_contour = cv2.drawContours(img, contours, idx, (255, 0, 255), 3)

# Draw a bounding box around the contour
x, y, w, h = cv2.boundingRect(biggest_contour)
cv2.rectangle(largest_contour, (x, y), (x + w, y + h), (255, 0, 0), 5)

# Crop the image to get rid of noise for future color segmentation
crop_ratio = 0.25
ratio = int(crop_ratio*h)

cropped_img[:y + ratio, :] = 0
cropped_img[y + h - ratio:, :] = 0
cropped_img[:, :x] = 0
cropped_img[:, x + w:] = 0

# Show the cropped image
cv2.imshow('cropped', cropped_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
