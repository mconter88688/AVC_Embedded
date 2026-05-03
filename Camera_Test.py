import Embedded as emb

TIME_STEP = 100

robo = emb.Robot()

cam = robo.getDevice("camera")
cam.enable(TIME_STEP)

print("Camera object created!!")

cam.showVideo()  # press q to quit
