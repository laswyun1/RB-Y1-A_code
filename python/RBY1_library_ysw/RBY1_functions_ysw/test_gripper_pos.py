# Test Right/Left Gripper Position values (Streaming until Ctrl+C)
# 
# Usage example:
#       (Remotely connect to Real RBY1-A robot UPC)
#       Before use this code, Robot should be powered ON
#       python3 test_gripper_pos.py
#       Freely move grippers and check angle position streaming
#       Default(Approximate): Closed(+90 deg), Open(-170 deg)
#
# Copyright (c) 2026 Precision Mechatronics Laboratory, KAIST. All rights reserved.
# Author: Seungwon Yun


# =============================================================================
# Import
# =============================================================================
import sys
from pathlib import Path
# folder_a(parent folder)/folder_b - file.py(current file)  ->   0: folder_b / 1: folder_a
sys.path.append(str(Path(__file__).resolve().parents[1]))
import rby1_lib_ysw as ysw

import time



# =============================================================================
# Code
# =============================================================================
ysw.setupLogging()

robotConfig = ysw.setRobotConfig(ysw.MODE_DIRECT_UPC, ysw.MODEL_A)
robot = ysw.initialize(robotConfig.mode, robotConfig.address, robotConfig.model, option=ysw.CONNECT_ONLY)

print("[Gripper Angle Position Test] Starts")

# Connect
gripper = ysw.Gripper()
gripper.connect(device_name=ysw.GRIPPER_DEVICE_NAME, protocol_version=ysw.DXL_PROTOCOL_VERSION, baudrate=ysw.DXL_BAUDRATE)
gripper.test_ping()

# Disable Torque Mode (Make it move freely by external force)
gripper.disable(dxl_id=ysw.GRIPPER_ID_R)
gripper.disable(dxl_id=ysw.GRIPPER_ID_L)

# Streaming angle position
try:
    while True:
        deg_R = gripper.read_position_angular(ysw.GRIPPER_ID_R, unit=ysw.DEG, print_log=True)
        deg_L = gripper.read_position_angular(ysw.GRIPPER_ID_L, unit=ysw.DEG, print_log=True)
        time.sleep(0.1)
    
except KeyboardInterrupt:
    print("\n[Gripper Angle Position Test] Ends")
    
finally:
    pass