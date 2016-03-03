# Script to capture an image process it looking for a certain color
# This shows all of the intermediate steps

import io
import time
import picamera
import picamera.array
import cv2
import numpy as np

#stream = io.BytesIO()
#with picamera.PiCamera() as camera:
#    with picamera.array.PiRGBArray(camera) as output:
#        camera.resolution = (640,480)
#        camera.capture(output, 'bgr')
#        
#        image = output.array

image = cv2.imread('blocks.jpg')

cv2.imshow('Image', image)

blurred = cv2.GaussianBlur(image, (11, 11), 0)
cv2.imshow('Blurred', blurred)



hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

greenLower = (29, 86, 6)
greenUpper = (90, 255, 255)


mask_green = cv2.inRange(hsv, greenLower, greenUpper)

cv2.imshow('Green Mask', mask_green)

blueLower = (90, 86, 6)
blueUpper = (130, 255, 255)

mask_blue = cv2.inRange(hsv, blueLower, blueUpper)

cv2.imshow('Blue Mask', mask_blue)

redLower = (0, 120, 6)
redUpper = (10, 255, 255)
redLower2 = (170, 120, 6)
redUpper2 = (180, 255, 255)

mask_red = cv2.bitwise_or(cv2.inRange(hsv, redLower, redUpper), 
                          cv2.inRange(hsv, redLower2, redUpper2))
mask_red = cv2.erode(mask_red, None, iterations=2)
mask_red = cv2.dilate(mask_red, None, iterations=2)
cv2.imshow('Red Mask', mask_red)


# find contours in the mask and initialize the current
# (x, y) center of the ball
cnts = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)[-2]
center = None

# only proceed if at least one contour was found
if len(cnts) > 0:
    # find the largest contour in the mask, then use
    # it to compute the minimum enclosing circle and    
    # centroid
    c = max(cnts, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    M = cv2.moments(c)
    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    
    # only proceed if the radius meets a minimum size
    if radius > 10:
        # draw the circle and centroid on the frame,
        # then update the list of tracked points
        cv2.circle(image, (int(x), int(y)), int(radius),
                          (0, 255, 255), 2)
        cv2.circle(image, center, 5, (0, 0, 255), -1)
cv2.imshow('Image', image)

k = cv2.waitKey(0) & 0xFF

cv2.waitKey(0)
        