#!/usr/bin/env python
import cv2
import numpy

# read png image and convert the image to HSV
image = cv2.imread("/home/fairchildc/Desktop/mission_scene.png", cv2.CV_LOAD_IMAGE_COLOR)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  

# find green objects in the image
lower_green = numpy.array([45, 110, 75], numpy.uint8)
upper_green = numpy.array([65, 210, 255], numpy.uint8)
mask = cv2.inRange(hsv, lower_green, upper_green)

cv2.imwrite("hsv_mask2.png", mask)

