import Embedded as emb
import cv2
import numpy as np

TIME_STEP = 100

robo = emb.Robot()

cam = robo.getDevice("camera")
cam.enable(TIME_STEP)

print("Camera object created!!")

# cam.showVideo()  # press q to quit

image = cam.getImage()
# Convert bytes to a 1D unsigned 8-bit integer array
nparr = np.frombuffer(image, np.uint8)

# Decode into an image (OpenCV format)
img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# Now you can use it like any other OpenCV image
cv2.imshow('Image', img)
#!cv2.waitKey(0) # until u click it its supposed to go away?
