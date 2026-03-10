## THINGS WE NEED TO FIX:

# Class: Robot:
# Methods:
# - .getBasicTimeStep()
# - getDevice("camera")
# - robot.step(TIME_STEP)

# class output of getDevice
# methods:
# - enable(TIME_STEP)
# - .setPosition(float('inf'))
# - .setvelocity(float('inf'))
# - .getImage()
# - .getWidth()
# - getHeight()
#  - .getRollPitchYaw()[2]
# - .getValue()

import smbus
import time


class Devices:
    DEFAULT_TIMESTEP = 32
    LEFT_MOTORS = "left wheel motor"
    RIGHT_MOTORS = "right wheel motor"
    CAMERA = "camera"
    IMU = "inertial unit"


class Robot:
    def __init__(self):
        self.timestep = Devices.DEFAULT_TIMESTEP
    def getBasicTimeStep(self):
        return self.timestep
    def getDevice(self, name):
        if name == Devices.LEFT_MOTORS or name == Devices.RIGHT_MOTORS:
            return Motor(name)
        elif name == Devices.IMU:
            return
        elif name == Devices.CAMERA:
            return
        else:
            raise ValueError("Invalid Device Type")


class Device:
    def __init__(self, name: str) -> None:
        self.name = name

class Motor:
    def __init__(self,name):
        self.name = name
    def setPosition(self, position_value: float):
        pass


class Imu(Device):
    def __init__(self, name: str):
        super().__init__(name)
        
