# Right/Left Gripper Motions by Current-based Position Control
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

print("[Gripper Current-based Position Control] Starts")

gripper = ysw.Gripper()
gripper.connect(device_name=ysw.GRIPPER_DEVICE_NAME, protocol_version=ysw.DXL_PROTOCOL_VERSION, baudrate=ysw.DXL_BAUDRATE)
gripper.test_ping()

### Initial setting
gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE)
gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_L, operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE)    
    
### Input desired positions(0mm - 91mm)
try:
    while True:
        target_position = float(input("Enter the Desired Position[mm]: "))
        if (target_position > 91.0 or target_position < 0.0):
            print("Desired Position should be in [0mm, 91mm]")
        else:
            # You can change Current[A] value LESS than ysw.DXL_CURRENT_LIMIT_MAX_A_YSW
            current = 0.02      # [A]
            gripper.current_based_position_control_linear(ysw.GRIPPER_ID_R, current_ampere_mag=current, linear_pos=target_position)
            gripper.current_based_position_control_linear(ysw.GRIPPER_ID_L, current_ampere_mag=current, linear_pos=target_position)
except KeyboardInterrupt:
    pass
finally:
    gripper.disable(ysw.GRIPPER_ID_R)
    gripper.disable(ysw.GRIPPER_ID_L)
    print("\n[Gripper Current-based Position Control] Ends")