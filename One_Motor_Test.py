import time
import Embedded as emb


left_motor = emb.robot.getDevice("left wheel motor")

print("Motor object created!!")


while(1):
    velocity_val = float(input("Velocity: "))

    left_motor.set_velocity(velocity_val)
    time.sleep(2)
    left_motor.set_velocity(0)
    time.sleep(1)
    left_motor.set_velocity(-velocity_val)
    time.sleep(2)
    left_motor.set_velocity(0)

