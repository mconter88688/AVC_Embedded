import time
import Embedded as emb

robo = emb.Robot()
left_motor = robo.getDevice("left wheel motor")

robo2 = emb.Robot()
right_motor = robo2.getDevice("right wheel motor")

print("Motor object created!!")


while(1):
    velocity_val = float(input("Velocity: "))

    left_motor.setVelocity(velocity_val)
    right_motor.setVelocity(0)
    time.sleep(2)
    left_motor.setVelocity(0)
    time.sleep(2)
    left_motor.setVelocity(0)
    right_motor.setVelocity(velocity_val)
    time.sleep(2)
    left_motor.setVelocity(-velocity_val)
    right_motor.setVelocity(0)
    time.sleep(2)
    left_motor.setVelocity(0)
    right_motor.setVelocity(-velocity_val)
    time.sleep(2)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)


