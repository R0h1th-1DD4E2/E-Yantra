# import the necessary packages
from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2

def frame_resize(frame, scale=.75):
    height=int(frame.shape[0]*scale)
    width=int(frame.shape[1]*scale)
    dimensions=(width,height)
    return cv2.resize(frame, dimensions)

# load the image, 
image = cv2.imread('led.jpg', 1)
image = frame_resize(image)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (11, 11), 0)

# threshold the image to reveal light regions in the blurred image
ret,thresholded = cv2.threshold(blurred,127,255,cv2.THRESH_BINARY)

thresh = cv2.erode(thresholded, None, iterations=2)
thresh = cv2.dilate(thresh, None, iterations=4)

# perform a connected component analysis on the thresholded image, then initialize a mask to store only the "large" components
labels = measure.label(thresh, connectivity=2, background=0)
mask = np.zeros(thresh.shape, dtype="uint8")

# loop over the unique components
for label in np.unique(labels):

# if this is the background label, ignore it
    if label == 0:
        continue 
    labelMask = np.zeros(thresh.shape, dtype="uint8")
    labelMask[labels == label] = 255
    numPixels = cv2.countNonZero(labelMask)
	# if the number of pixels in the component is sufficiently large, then add it to our mask of "large blobs"
    if numPixels > 400:
        mask = cv2.add(mask, labelMask)

contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

# Initialize lists to store centroid coordinates and area
centroids = []
areas = []
cv2.waitKey(10)
for i, j in enumerate(contours):

    # Calculate the area of the contour
    area = cv2.contourArea(j)
    cv2.drawContours(image, [j], -1, (0, 255, 0), 2)
    # Calculate the moments of the contour
    M = cv2.moments(j)
    # Append centroid coordinates and area to the respective lists
    centroidx = float(M["m10"] / M["m00"])
    centroidy = float(M["m01"] / M["m00"])
    centroids.append((centroidx, centroidy))
    areas.append(area)
# Save the output image as a PNG file
cv2.imwrite("led_detection_results.png", image)

# Open a text file for writing
with open("led_detection_results.txt", "w") as file:
    # Write the number of LEDs detected to the file
    file.write(f"No. of LEDs detected: {len(contours)}\n")
    # Loop over the contours
    for i, (centroid, area) in enumerate(zip(centroids, areas)):
        # Write centroid coordinates and area for each LED to the file
        file.write(f"Centroid #{i + 1}: {centroid}\nArea #{i + 1}: {area}\n")
# Close the text file
file.close()
cv2.waitKey(10)
