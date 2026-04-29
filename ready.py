import rby1_sdk as rby
import functions as fn


ROBOT_ADDRESS = "192.168.30.1:50051"        # For Real Robot
robot = rby.create_robot_a(ROBOT_ADDRESS)   # Robot Model is "A"
model = robot.model()


if not robot.connect():
    print("FAIL: Could not connect to robot")
    exit(1)
else:
    print("Start to move READY position")
    fn.move_position(robot, fn.READY_POS)
    print("End to move READY position")