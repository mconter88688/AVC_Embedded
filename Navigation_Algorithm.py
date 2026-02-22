"""
Date: 2/22/2026
Purpose: Python navigation algorithms, utilizing object detection to navigate an obstacle course consisting of one blue buckets, red two buckets with plywood on top, five yellow buckets, 
         and a ramp. Turn left around blue buckets, trun right around yellow buckets, and go straight through the yellow buckets."""

from controller import Robot, Camera, Motor, InertialUnit, PositionSensor
from ultralytics import YOLO
import numpy as np
import cv2
import math
import time
from enum import Enum

# ------------INITIALIZATION------------------

# Initialize the Robot
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

# Initialize Camera
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)


# Initialize Motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# Set motors to velocity control mode
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Initialize IMU (Inertial Unit)
imu = robot.getDevice("inertial unit")
imu.enable(TIME_STEP)

# Encoders
left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

# Load YOLO model
try:
    model = YOLO("best1.onnx", task="detect")
    print("YOLO model loaded successfully")
except Exception as e:
    print(f"Error loading YOLO model: {e}")

# ----------- CONFIGURATION -------------------

CONFIDENCE_THRESHOLD = 0.10

# Physical properties
# FOCAL_LENGTH = (pixel_width * distance) / real_width
REAL_BUCKET_WIDTH = 0.3  # Meter?? 
FOCAL_LENGTH = 500       # Pixel units 

# Navigation Constants
STOP_DISTANCE = 0.5      # Meter?? 
BASE_SPEED = 5.0         
KP = 2.0                 # Runing Value: How aggressively to turn during steering; Too low -> sluggish, too high --> jerky

# Calculating offset for steering
BLUE_OFFSET = -0.3        # Keep bucket on the LEFT side of the image.
YELLOW_OFFSET = 0.3       # Yellow: We want to pivot Right, so we drive to the Left of it. Keep bucket on the RIGHT side of the image.
RED_OFFSET = 0.0          # Red: Drive straight through.

# Encoder / wheel placeholders
TICK_PER_MOTOR = 2 
GEAR_RATIO = 2 
TICKS_PER_WHEEL_REVOLUTION = TICK_PER_MOTOR * GEAR_RATIO
WHEEL_RADIUS = 0.0205
WHEEL_CIRCUMFERENCE = 2 * math.pi * WHEEL_RADIUS
DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_WHEEL_REVOLUTION

# Distance to drive by after the STOP_DISTANCE
BLUE_DRIVEPASS_DIST = 1
YELLOW_DRIVEPASS_DIST = 1
RED_DRIVEPASS_DIST = 1

# Bucket Logic
class colors(Enum):
    blue = 0
    red = 1
    yellow = 2

# Sequence
COURSE_BUCKET_ORDER = [
    colors.yellow,
    colors.yellow,
    colors.blue,
    colors.yellow,
    colors.yellow,
    colors.red,
    colors.yellow
]

# --------------DETECTION FUNCTION-----------------

def get_bucket_of_certain_color_location(found_buckets, color_id):
    '''
    HELPER function to find the pixel location of a specific bucket color within the frame.
    
    Inputs:
       - found_buckets: List that contains the classes filtered by colors of the objects detected
       - color_id: the id number associated with each bucket color; Each color unique class ID: 0 = blue, 1 = red, 2 = yellow
    
    Return:
        - center_x_scaled: Center of the bucket from -1.0 (left) to 1.0 (right).
        - distance: Calculate exact distance based on formula --> (focal_length * real_width) / pixel_width
        - bucket_detected: True or False, whether bucket is detected
   '''

    if color_id in found_buckets:

        if color_id == colors.red: # If looking for red, must find two and calculate average distance to get center position
            if len(found_buckets) == 2:
                red1 = found_buckets[0]
                red2 = found_buckets[1]
                midpoint = (red1['x'] + red2['x']) / 2 # Center pixel based on middle of two red buckets

                avg_width = (red1['x'] + red2['x']) / 2
                dist = (FOCAL_LENGTH * REAL_BUCKET_WIDTH) / avg_width # Distance based on middle of two red buckets
                print(f"Bucket detected: Middle_Center_X: {midpoint:.3f}, Distance: {dist}" )
                return midpoint, dist, True
            
            else: 
                pass # Something if only can see one bucket

        else: # If blue or yellow, find the closest bucket (One with largest bounding box)
            if found_buckets: # If not empty
                largest = max(found_buckets, key=lambda b: b['w'])
                dist = (FOCAL_LENGTH * REAL_BUCKET_WIDTH) / largest['w']
                print(f"Bucket detected: Center_X: {largest['x']:.3f}, Distance: {dist}" )
                return largest['x'], dist, True
    
    else:
        return [0, 0, False] 


def detect_buckets(color_id):    
    '''
    Scans the camera frame to get image, runs YOLO detection, and finds the target bucket.
    
    Inputs:
       - color_id: the id number associated with each bucket color; Each color unique class ID: 0 = blue, 1 = red, 2 = yellow

    Returns:
        - center_x_scaled: Center of the bucket from -1.0 (left) to 1.0 (right).
        - distance: Calculate exact distance based on formula --> (focal_length * real_width) / pixel_width
        - bucket_detected: True or False, whether bucket is detected
    '''
    image = camera.getImage()
    width, height = camera.getWidth(), camera.getHeight()
    img = np.frombuffer(image, np.uint8).reshape((width, height, 4))
    frame = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR) # Fix color conversion between YOLO and webot
    cv2.imshow("Detected Objects", frame)
    print(f"Processing frame size: {frame.shape}")
    
    resized_frame = cv2.resize(frame, (416, 416)) # Resize
    results = model(resized_frame, conf=CONFIDENCE_THRESHOLD) # Run Model
    scale_x = width / 416
    scale_y = height / 416
    cv2.waitKey(1)
    
    for result in results:
        bucket_founds = [] # Create an empty list to store classes of object detected
        boxes = result.boxes # List of bounding boxes found in images

        for box in boxes: # Each box contain coordinates (x1, y1, x2, y2), predicted class (cls), and confidence score (conf)
            predicted_class = int(box.cls[0].item()) # Put the predicted class as int, not float 
            conf = box.conf[0].item()   
            print(f"SEES: Class {int(box.cls[0].item())} with confidence {box.conf[0].item():.2f}")
            if predicted_class == color_id:   # Filter only the colors we are looking for 
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x_pixels = (x1 + x2) / 2 # Get the horizontal center of bucket in the image
                center_x_scaled = (center_x_pixels / 208) - 1
                width_pixels = x2 - x1
                print(f"Bucket detected: Class {predicted_class}, Center_X: {center_x_scaled:.3f}, Width: {width_pixels}, Confidence: {conf:.2f}" )
                bucket_founds.append({'x': center_x_scaled, 'w': width_pixels})
                
            # Rescale back to original frame size
                x1 = int(x1 * scale_x)
                y1 = int(y1 * scale_y)
                x2 = int(x2 * scale_x)
                y2 = int(y2 * scale_y)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"Class {predicted_class}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
            
        found_x, found_dist, found_bucket = get_bucket_of_certain_color_location(bucket_founds, color_id) # Only look once loop is done

        if found_bucket == True:
            return  found_x, found_dist, found_bucket
            
    print("Error: Frame could not be read. Attempts stop")   
    return None, None, False # If no bucket found, along with its location, depsite 10 try, return None       
 
# ---------------MOVEMENT FUNCTION------------------

def set_motors_speed(left_speed, right_speed):
    '''Run the motors with the given speeds'''
    # Set velocity. Same value -> go forward
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

def stop_motors():
    '''Stops velocity'''
    set_motors_speed(0.0, 0.0)

def steer_toward_bucket(color_id, center_x, base_speed):
    """
    Given scaled_x_position, lock on the target and adjust left/rght speed to keep on track using diffrential steering as to move towards it: 
    Left_Speed > Right_Speed mean turning right, the difference determines sharpness or smoothness of speed. Less -> smooth, Greater -> Sharper. Vice versa.

    Inputs:
    - center_x_scaled: Center of the bucket from -1.0 (left) to 1.0 (right).
    - target_offse: Steer towards the bucket; 0 
    - base_speed: Default speed
    """   
    # Calculate how off we are from desired position and steer accordingly
    if color_id == colors.blue:
        target_offset = BLUE_OFFSET
    elif color_id == colors.yellow:
        target_offset = YELLOW_OFFSET
    elif color_id == colors.red:
        target_offset = RED_OFFSET
    else:
        print(f"steer_toward_bucket: Target bucket {color_id} not available")
        return # Exit if unknown color id    

    error = center_x - target_offset
    correction = error * KP

    left_speed = base_speed + correction
    right_speed = base_speed - correction
    if left_speed > right_speed:
        print(f"Steering  right")
    elif left_speed < right_speed:
        print(f"Steering  left")
    else:
        print(f"Steering  forward")
    set_motors_speed(left_speed, right_speed)



def pivot_turn(degree_of_pivot, base_speed):
    '''
    Rotates the robot using IMU feedback.

    Input:
    - degree_of_pivot: degrees > 0: Turn Left, degrees < 0: Turn Right
    - base_speed
    '''
    start_yaw = imu.getRollPitchYaw()[2] # Accessed yaw (radians) to know direction facing
    target = start_yaw + math.radians(degree_of_pivot) # Calculate the target direction after condisering initial and pivot degree

    while robot.step(TIME_STEP) != -1:
        yaw = imu.getRollPitchYaw()[2] # Access new yaw, and compare it to target 
        error = target - yaw

        if abs(error) < math.radians(2): # allows tolerance; Provided by ChatGPT
            break
        
        # Positive, turn left. Negative, turn right
        if error > 0:
            turn_speed = base_speed 
        else:
            turn_speed = - base_speed 

        # Spin
        set_motors_speed(-turn_speed, turn_speed)

    stop_motors()    

def drive_pass(target_distance, base_speed): # Using DC gear motor with built-in encoder? Contionus servo don't have way to measure accurate distance travvled
    """Drive straight forward for a fixed distance using wheel encoders"""
    # Start wheel rotation and angle
    left_start = left_encoder.getValue()
    right_start = right_encoder.getValue()
    target_yaw = imu.getRollPitchYaw()[2]


    while robot.step(TIME_STEP) != -1:
        left_now = left_encoder.getValue()
        right_now = right_encoder.getValue()

        # Distance= radian angle × radius
        delta_left = (left_now - left_start) * WHEEL_RADIUS
        delta_right = (right_now - right_start) * WHEEL_RADIUS

        # Average distance traveled, to factor for potential  difference
        avg_distance = (delta_left + delta_right) / 2

        # If drove close enough to target distance, then stop. Else, move forward
        if avg_distance >= target_distance:
            break

        current_yaw = imu.getRollPitchYaw()[2]  
        yaw_error = current_yaw - target_yaw

        set_motors_speed(base_speed, base_speed)

    stop_motors()


 # ---------------- MAIN LOGIC ----------------
   
def toward_bucket(color_id, course_number):

    while robot.step(TIME_STEP) != -1: # This makes the robot wait for the next frame

        # Get center_x and pixel_width if bucket_detected
        center_x, distance, bucket_detected = detect_buckets(color_id) 

        if bucket_detected == True:
            print(f"Distance: {distance}, ")

            if distance > STOP_DISTANCE: # If distance still > 1.5 ft (18 inches), continue moving
                print(f"Driving...{distance}")
                steer_toward_bucket(color_id, center_x, BASE_SPEED)        

            else:
                print("Arrived at stop! Now, driving pass and pivoting!")
                stop_motors()  

                # Drive pass and Pivot Logic
                # If yellow, pivot (pivot degree varies depedping on what stage)
                if color_id == colors.yellow:
                    if course_number == 1:
                        drive_pass(YELLOW_DRIVEPASS_DIST,BASE_SPEED)
                        pivot_turn(-80)
                    elif course_number == 2:
                        drive_pass(YELLOW_DRIVEPASS_DIST,BASE_SPEED)
                        pivot_turn(-60)
                    elif course_number == 4:
                        drive_pass(YELLOW_DRIVEPASS_DIST,BASE_SPEED)
                        pivot_turn(-55)
                    elif course_number == 5:
                        drive_pass(YELLOW_DRIVEPASS_DIST,BASE_SPEED)
                        pivot_turn(-60)
                    elif course_number == 8:
                        drive_pass(YELLOW_DRIVEPASS_DIST,BASE_SPEED)
                        pivot_turn(-90)

                # If red, no pivot
                elif color_id == colors.red:
                    drive_pass(RED_DRIVEPASS_DIST,BASE_SPEED)
                
                # If blue, two drive pass and pivot
                elif color_id == colors.blue:
                    drive_pass(BLUE_DRIVEPASS_DIST,BASE_SPEED)
                    pivot_turn(90)
                    drive_pass(BLUE_DRIVEPASS_DIST,BASE_SPEED)
                    pivot_turn(100)
                    drive_pass(BLUE_DRIVEPASS_DIST,BASE_SPEED)
        
                print(f"Bucket {color_id}, course numer {course_number} is finished.Moving on bucket {COURSE_BUCKET_ORDER[course_number]}, course number {course_number+1}") 
                break # Move on the next bucket

        else:
            print(f"Still scanning for {color_id}...")


def main():
    print("STARTING COURSE")
    
    # enumerate gives both the index and the color id
    for index, target_color in enumerate(COURSE_BUCKET_ORDER):
        course_number = index + 1
        print(f"\n STARTING TASK {course_number}: {target_color.name}")

        # Completing target bucket
        toward_bucket(target_color.value, course_number)
        
        # Target Task Complete
        print(f"\n TASK {course_number} COMPLETE")
        time.sleep(1.0) 


    print("ALL TASKS FINISHED. STOPPING.")
    stop_motors()
    cv2.destroyAllWindows()

main()
