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

'''
INSTRUCTIONS ON SCPING THINGS:
1. first ssh into the pi: ssh avc@172.20.10.12
2. cd into the AVC_Embedded folder
3. open a new terminal window not ssh-ed into
4. cd into the AVC_Embedded folder into the nurobotics folder
5. then scp a folder INTO the pi via the following commands:

if scping a file: in the below example, you need to be inside the AVC_Embedded directory. youll scp the embedded.py file into the pi

scp Embedded.py avc@172.20.10.12:~/Documents/embedded_2026/nurobotics/AVC_Embedded/




'''
import board
import busio
import math
import adafruit_lsm9ds1
import time
from picamera2 import Picamera2
from gpiozero import PWMOutputDevice
import cv2

LEFT_MOTOR_1_FORWARD = 24 #18
LEFT_MOTOR_1_REVERSE = 10 #19
LEFT_MOTOR_2_FORWARD = 23
LEFT_MOTOR_2_REVERSE = 22
RIGHT_MOTOR_1_FORWARD = 14
RIGHT_MOTOR_1_REVERSE = 15
RIGHT_MOTOR_2_FORWARD = 17
RIGHT_MOTOR_2_REVERSE = 18

LEFT_MOTOR_FORWARD = [LEFT_MOTOR_1_FORWARD, LEFT_MOTOR_2_FORWARD]
LEFT_MOTOR_REVERSE = [LEFT_MOTOR_1_REVERSE, LEFT_MOTOR_2_REVERSE]
RIGHT_MOTOR_FORWARD = [RIGHT_MOTOR_1_FORWARD, RIGHT_MOTOR_2_FORWARD]
RIGHT_MOTOR_REVERSE = [RIGHT_MOTOR_1_REVERSE, RIGHT_MOTOR_2_REVERSE]

MAX_SPEED = 10

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
        if name == Devices.LEFT_MOTORS:
            return Motor(name, LEFT_MOTOR_FORWARD, LEFT_MOTOR_REVERSE, MAX_SPEED)
        elif name == Devices.RIGHT_MOTORS:
            return Motor(name, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_REVERSE, MAX_SPEED)
        elif name == Devices.IMU:
            return Imu(name)
        elif name == Devices.CAMERA:
            return Camera(name)
        else:
            raise ValueError("Invalid Device Type")


class Device:
    def __init__(self, name: str) -> None:
        self.name = name

class Motor(Device):
    def __init__(self,name, forward_gpio, reverse_gpio, MAX_SPEED):
        
        
        
        self.forward_gpio = forward_gpio
        self.reverse_gpio = reverse_gpio
        self.MAX_SPEED = MAX_SPEED

        self.forward_pwm = [
            PWMOutputDevice(pin, frequency=1000) for pin in forward_gpio
        ]
        self.reverse_pwm = [
            PWMOutputDevice(pin, frequency=1000) for pin in reverse_gpio
        ]


    def setPosition(self, position_value: float):
        pass
    def setVelocity(self, velocity_value: float):
        velocity = max(min(velocity_value, self.MAX_SPEED), -self.MAX_SPEED)
        duty_cycle = abs(velocity) / self.MAX_SPEED

        if velocity > 0:
            for f_pwm, r_pwm in zip(self.forward_pwm, self.reverse_pwm):
                f_pwm.value = duty_cycle
                r_pwm.value = 0

        elif velocity < 0:
            for f_pwm, r_pwm in zip(self.forward_pwm, self.reverse_pwm):
                f_pwm.value = 0
                r_pwm.value = duty_cycle

        else:
            for f_pwm, r_pwm in zip(self.forward_pwm, self.reverse_pwm):
                f_pwm.value = 0
                r_pwm.value = 0

class Imu(Device):

    def __init__(self, name: str):
        super().__init__(name)
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)



    def enable(self, timestep):
        pass

    def getRollPitchYaw(self):

        accel_x, accel_y, accel_z = self.sensor.acceleration
        mag_x, mag_y, mag_z = self.sensor.magnetic

        
        roll = math.atan2(accel_y, accel_z)
        pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))

        mag_x_comp = mag_x * math.cos(pitch) + mag_z * math.sin(pitch)

        mag_y_comp = (
            mag_x * math.sin(roll) * math.sin(pitch)
            + mag_y * math.cos(roll)
            - mag_z * math.sin(roll) * math.cos(pitch)
        )

        yaw = math.atan2(-mag_y_comp, mag_x_comp)

        return (roll, pitch, yaw)

class Camera(Device):
    def __init__(self, name: str):
        super().__init__(name)

        # opencv camera object
        self.cam = None

        # resolution
        self.width = 640
        self.height = 480

    def enable(self, timestep):
        
        # initialize the Raspberry Pi camera
        self.cam = cv2.VideoCapture(0)
        
        # Set resolution to 1080p (optional but recommended for USBFHD04H)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # Check if initialized
        if not self.cam.isOpened():
            print("Cannot open camera")
            exit()



        # self.cam = Picamera2()
        # # configure resolution
        # config = self.cam.create_preview_configuration(
        #     main={
        #         "size": (self.width, self.height),
        #         "format": "RGB888"
        #     }
        # )

        # self.cam.configure(config)

        # # start the camera
        # self.cam.start()


    def getImage(self):
        """
        capture one frame from the camera
        """
        image_captured = False
        while not image_captured:
            ret, frame = self.cam.read()
            if ret:
                image_captured = True
        image_bytes = frame.tobytes()
        return image_bytes
        
        
        
        '''
            cv2.imshow('USBFHD04H Feed', frame)
            
            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Clean up
        self.cam.release()
        cv2.destroyAllWindows() '''
        
    #! JORDAN CODE ==============
        # # capture frame from camera
        # frame = self.cam.capture_array()

        # if frame is None:
        #     print("Camera failed to capture image")
        #     return None

        # # convert the frame to raw bytes
        # image_bytes = frame.tobytes()

        # return image_bytes
        #!=======================

    def getWidth(self):
        return self.width

    def getHeight(self):
        return self.height

    def disable(self):
        # stop the camera when finished

        if self.cam is not None:
            self.cam.stop()

    def showVideo(self):
        """
        debug function to display camera feed
        press 'q' to exit
        """

        while True:

            frame = self.cam.capture_array()

            cv2.imshow("Camera", frame)

            if cv2.waitKey(1) == 113: # 113 is ASCII representation 'q'
                break

        cv2.destroyAllWindows()
