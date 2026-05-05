import rby1_sdk as rby
import functions as fn


ROBOT_ADDRESS = "192.168.30.1:50051"        # For Real Robot
robot = rby.create_robot_a(ROBOT_ADDRESS)   # Robot Model is "A"
model = robot.model()


if not robot.connect():
    print("FAIL: Could not connect to robot")
    exit(1)
else:
    rotation_angle = float(input("Enter the desired rotation[deg]\n(+)Left, (-)Right: "))
    print("Start to Turn")
    fn.rotate_inplace_smooth(robot, rotation_angle)     
    print("End to Turn")