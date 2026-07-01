# Right/Left Gripper Homing
# 
# Usage example:
#       (Remotely connect to Real RBY1-A robot UPC)
#       Before use this code, Activate "python3 initialize.py"
#       python3 gripper_homing.py
#       This function is only used when the new homing is needed (After the setting of homing, you don't have to use this periodically)
#       After running this code, YOU MUST change the variable values in "robot_info.py" file. (Check the log of this code in terminal)
#       During this code execution, Gripper should be fully OPEN and CLOSED. 
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

print("[Gripper Homing] Starts")

gripper = ysw.Gripper()
gripper.connect(device_name=ysw.GRIPPER_DEVICE_NAME, protocol_version=ysw.DXL_PROTOCOL_VERSION, baudrate=ysw.DXL_BAUDRATE)
gripper.test_ping()

### Initial setting is Done in "homing" function
gripper.homing(ysw.GRIPPER_ID_R, iter=100)
gripper.homing(ysw.GRIPPER_ID_L, iter=100)

### Deactivation After Homing
gripper.disable(ysw.GRIPPER_ID_R)
gripper.disable(ysw.GRIPPER_ID_L)

print("[Gripper Homing] Ends")