import rby1_sdk as rby
import time


ROBOT_ADDRESS = "192.168.30.1:50051"        # For Real Robot
robot = rby.create_robot_a(ROBOT_ADDRESS)   # Robot Model is "A"
model = robot.model()


if not robot.connect():
    print("FAIL: Could not connect to robot")
    exit(1)
else:
    robot.set_tool_flange_output_voltage("right", 0)
    robot.set_tool_flange_output_voltage("left", 0)
    time.sleep(1)
    robot.disable_control_manager()
    robot.power_off(".*")  
    print("SUCCESS: Power OFF the robot!!")
    
    