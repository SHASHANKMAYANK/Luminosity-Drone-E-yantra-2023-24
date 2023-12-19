# import the necessary packages
from skimage import measure
from imutils import contours
import numpy as np
import imutils
import cv2 as cv

# load the image,
img = cv.imread('led.jpg', 1)

# convert it to grayscale, and blur it
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
bilateral = cv.bilateralFilter(gray, 1, 15, 15)
# threshold the image to reveal light regions in the blurred image
ret, thresh = cv.threshold(bilateral, 105, 255, cv.THRESH_BINARY)

# perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image
dilated = cv.dilate(thresh, (3, 3), iterations=3)
eroded = cv.erode(dilated, (5, 5), iterations=3)

# perform a connected component analysis on the thresholded image, then initialize a mask to store only the "large" components


# loop over the unique components

# if this is the background label, ignore it

# otherwise, construct the label mask and count the number of pixels

# if the number of pixels in the component is sufficiently large, then add it to our mask of "large blobs"

blank = np.zeros(img.shape[:2], dtype='uint8')
b, g, r = cv.split(img)
# find the contours in the mask, then sort them from left to right
contours, hierarchies1 = cv.findContours(eroded, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
contours1, hierarchies1 = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
# loop over the contours

# Initialize lists to store centroid coordinates and area

# Loop over the contours


# Calculate the area of the contour


# Draw the bright spot on the image


# Append centroid coordinates and area to the respective lists

areas = []
centroid = []
led = 0

cv.drawContours(blank, contours1, -1, (255, 255, 255), 8)
bitwise_add = cv.bitwise_or(r, blank)
merged = cv.merge([b, g, bitwise_add])
for i in range(0, len(contours), 1):
    cnt = contours[i]
    M = cv.moments(cnt)
    cx = M['m10'] / M['m00']
    cy = M['m01'] / M['m00']
    area = cv.contourArea(cnt)
    areas.append(area)
    centroid.append((cx, cy))
    led += 1
    text = f"+{led}"
    cv.putText(merged, text, (int(cx) - 20, int(cy) - 30), cv.FONT_HERSHEY_COMPLEX, 0.8, (0, 0, 255), 5, cv.LINE_AA)

# Save the output image as a PNG file
filename = 'LD_1566_led_detection_results.png'
cv.imwrite(filename, merged)

# Open a text file for writing
file2 = open("LD_1566_led_detection_results.txt", "w+")
# Write the number of LEDs detected to the file
file2.write(f"No. of LEDs detected: {led}\n")
# Loop over the contours
for i in range(0, len(areas), 1):
    # Write centroid coordinates and area for each LED to the file
    file2.write(f"Centroid #{i + 1}: {centroid[i]}\nArea #{i + 1}: {areas[i]}\n")
# Close the text file
file2.close()
