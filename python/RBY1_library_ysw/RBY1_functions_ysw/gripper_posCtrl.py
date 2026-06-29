# Right/Left Gripper Motions by Extended Position Control
# 
# Usage example:
#       (Remotely connect to Real RBY1-A robot UPC)
#       Before use this code, Robot should be powered ON
#       python3 gripper_posCtrl.py
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

print("[Gripper Position Control] Starts")

gripper = ysw.Gripper()
gripper.connect(device_name=ysw.GRIPPER_DEVICE_NAME, protocol_version=ysw.DXL_PROTOCOL_VERSION, baudrate=ysw.DXL_BAUDRATE)
gripper.test_ping()

### Initial setting
gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_EXTENDED_POSITION_CONTROL_MODE)
gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_L, operation_mode=ysw.DXL_EXTENDED_POSITION_CONTROL_MODE)    
    
### Input desired positions(0mm - 91mm)
try:
    while True:
        target_position = float(input("Enter the Desired Position[mm]: "))
        if (target_position > 91.0 or target_position < 0.0):
            print("Desired Position should be in [0mm, 91mm]")
        else:
            gripper.position_control_linear(ysw.GRIPPER_ID_R, target_position)
            gripper.position_control_linear(ysw.GRIPPER_ID_L, target_position)
except KeyboardInterrupt:
    pass
finally:
    print("\n[Gripper Position Control] Ends")