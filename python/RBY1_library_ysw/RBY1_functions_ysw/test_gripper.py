# Test Right/Left Gripper Motions by Current Control
# 
# Usage example:
#       (Remotely connect to Real RBY1-A robot UPC)
#       Before use this code, Robot should be powered ON
#       python3 test_gripper.py
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

print("[Gripper Test] Starts")

gripper = ysw.Gripper()
gripper.connect(device_name=ysw.GRIPPER_DEVICE_NAME, protocol_version=ysw.DXL_PROTOCOL_VERSION, baudrate=ysw.DXL_BAUDRATE)
gripper.test_ping()

### Initial setting method 1
# for dxl_id in gripper.dxl_activated:
#     gripper.write_torque_mode(dxl_id=dxl_id, torque_mode=ysw.DXL_TORQUE_DISABLE)
# #   gripper.write_current_limit(dxl_id=dxl_id, current_limit_A=ysw.DXL_CURRENT_LIMIT_MAX_A_GRIPPER)
#     gripper.write_operation_mode(dxl_id=dxl_id, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
#     gripper.write_torque_mode(dxl_id=dxl_id, torque_mode=ysw.DXL_TORQUE_ENABLE)
    
### Initial setting method 2
gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_L, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)    
    
trial = 5
for i in range(trial):
    print(f"[Trial {i+1}/{trial}]")
    gripper.write_current_ampere(ysw.GRIPPER_ID_R, -0.02)
    gripper.write_current_ampere(ysw.GRIPPER_ID_L, -0.02)
    time.sleep(2)
    gripper.write_current_ampere(ysw.GRIPPER_ID_R, 0.02)
    gripper.write_current_ampere(ysw.GRIPPER_ID_L, 0.02)
    time.sleep(2)
    
### Deactivation After Test
gripper.disable(ysw.GRIPPER_ID_R)
gripper.disable(ysw.GRIPPER_ID_L)

print("[Gripper Test] Ends")