# Move to ZERO Posture
# 
# Usage example:
#       (Remotely connect to Real RBY1-A robot UPC)
#       Before use this code, Robot should be powered ON
#       python3 poseZero.py
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



# =============================================================================
# Code
# =============================================================================
ysw.setupLogging()

robotConfig = ysw.setRobotConfig(ysw.MODE_DIRECT_UPC, ysw.MODEL_A)
robot = ysw.initialize(robotConfig.mode, robotConfig.address, robotConfig.model, option=ysw.CONNECT_ONLY)

print("[Posture: ZERO] Starts")
ysw.move_position_22dof(robot, ysw.ZERO_POS, minimum_time=5.0, control_hold_time=0.5, get_return=True)
print("[Posture: ZERO] Ends")