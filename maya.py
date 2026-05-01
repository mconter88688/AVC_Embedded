import board
import busio
import math
import time
from gpiozero import PWMOutputDevice

pin = PWMOutputDevice(10)
LEFT_MOTOR_1_FORWARD = 24 #18
LEFT_MOTOR_1_REVERSE = 10 #19
pin.value = 0.5
print("pin value", pin.value)
time.sleep(1000)

