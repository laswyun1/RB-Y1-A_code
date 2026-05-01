import rby1_sdk as rby
import time


ROBOT_ADDRESS = "192.168.30.1:50051"        # For Real Robot
robot = rby.create_robot_a(ROBOT_ADDRESS)   # Robot Model is "A"
model = robot.model()

if not robot.connect():
    print("FAIL: Could not connect to robot")
    exit(1)
else:
    print("SUCCESS: Connected to the robot!!")

    while True:
        ans = input("Power ON & Enable control manager? (y/n): ").strip().lower()
        if ans in ['y', 'n']:
            break
        print("Please enter 'y' or 'n'.")

    if ans == 'y':
        robot.power_on(".*")
        time.sleep(2)
        robot.servo_on(".*")
        time.sleep(2)
        robot.reset_fault_control_manager()
        robot.enable_control_manager(unlimited_mode_enabled=True)
        print("Robot is ready.")
    else:
        print("Skipped robot activation.")










