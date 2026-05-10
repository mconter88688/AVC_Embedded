import cv2
import numpy as np

# Example: Read a file into bytes
with open('test.jpg', 'rb') as f:
    byte_data = f.read()

# Convert bytes to a 1D unsigned 8-bit integer array
nparr = np.frombuffer(byte_data, np.uint8)

# Decode into an image (OpenCV format)
img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# Now you can use it like any other OpenCV image
cv2.imshow('Image', img)
cv2.waitKey(0) # until u click it its supposed to go away?
