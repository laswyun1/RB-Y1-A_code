import rby1_sdk as rby
import functions as fn
import numpy as np
import time


ROBOT_ADDRESS = "192.168.30.1:50051"        # For Real Robot
robot = rby.create_robot_a(ROBOT_ADDRESS)   # Robot Model is "A"
model = robot.model()

if not robot.connect():
    print("FAIL: Could not connect to robot")
    exit(1)
else:
    print("SUCCESS: Connect to the robot.")

fn.move_neck_example(robot)

