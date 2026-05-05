import rby1_sdk as rby
import functions as fn


ROBOT_ADDRESS = "192.168.30.1:50051"        # For Real Robot
robot = rby.create_robot_a(ROBOT_ADDRESS)   # Robot Model is "A"
model = robot.model()


if not robot.connect():
    print("FAIL: Could not connect to robot")
    exit(1)
else:
    displacement = float(input("Enter the desired distance[cm]\n(+)Forward, (-)Backward: "))
    displacement = displacement * 0.01
    print("Start to move")
    fn.move_forward_distance_smooth(robot, displacement)
    print("End to move")