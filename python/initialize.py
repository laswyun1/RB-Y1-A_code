import rby1_sdk as rby
import time
from dynamixel_sdk import PortHandler, PacketHandler
import functions as fn


ROBOT_ADDRESS = "192.168.30.1:50051"        # For Real Robot
robot = rby.create_robot_a(ROBOT_ADDRESS)   # Robot Model is "A"
model = robot.model()

if not robot.connect():
    print("FAIL: Could not connect to robot")
    exit(1)
else:
    print("SUCCESS: Connected to the robot!!")
    if (robot.power_on(".*") == True):         # Turn on all the power (5v, 12v, 24v, 48v)
        time.sleep(1)
        if (robot.servo_on(".*") == True):         # Enable servo motors on all actuators
            time.sleep(1)
            robot.reset_fault_control_manager()
            robot.enable_control_manager(unlimited_mode_enabled=True)
            print("SUCCESS: Power ON the Robot.")
            
            # 여기 추가: 그리퍼 쪽 12V 출력 ON
            time.sleep(1)
            robot.set_tool_flange_output_voltage("right", 12)
            robot.set_tool_flange_output_voltage("left", 12)
            print("SUCCESS: Power ON the Gripper.")
        else:
            print("Failed to power ON the Servo")   
