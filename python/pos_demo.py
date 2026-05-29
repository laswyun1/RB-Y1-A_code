import rby1_sdk as rby
import functions as fn


ROBOT_ADDRESS = "192.168.30.1:50051"        # For Real Robot
robot = rby.create_robot_a(ROBOT_ADDRESS)   # Robot Model is "A"
model = robot.model()


if not robot.connect():
    print("FAIL: Could not connect to robot")
    exit(1)
else:
    print("Start to move DEMO posture")
    fn.move_position_22dof(robot, fn.READY_DEMO_POS, minimum_time=5.0)
    print("End to move DEMO posture")