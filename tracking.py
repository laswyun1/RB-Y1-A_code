import rby1_sdk as rby
import time
import numpy as np
import functions as fn



ROBOT_ADDRESS = "192.168.30.1:50051"        # For Real Robot
# ROBOT_ADDRESS = "localhost:50051"           # For Docker Simulation
robot = rby.create_robot_a(ROBOT_ADDRESS)   # Robot Model is "A"
model = robot.model()


if not robot.connect():
    print("FAIL: Could not connect to robot")
    exit(1)
else:
    print("SUCCESS: Connected to the robot!!")
    
    
fn.move_forward_distance_smooth(robot, 2.5)         # Forward 2.5m
time.sleep(2)
fn.rotate_inplace_smooth(robot, -83)                # Right turn 90degree
time.sleep(2)
fn.move_forward_distance_smooth(robot, 1.5)         # Forward 1.5m   
time.sleep(2)
fn.rotate_inplace_smooth(robot, -83)                # Right turn 90degree
time.sleep(2)
fn.move_forward_distance_smooth(robot, 2.5)         # Forward 2.5m
time.sleep(2)
fn.rotate_inplace_smooth(robot, -83)                # Right turn 90degree
time.sleep(2)
fn.move_forward_distance_smooth(robot, 1.5)         # Forward 1.5m   
time.sleep(2)
fn.rotate_inplace_smooth(robot, -83)                # Right turn 90degree
time.sleep(2)
