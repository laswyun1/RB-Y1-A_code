# Initialize Robot with Unlimit mode (Connect + Power ON + Servo ON + Enable Control Manager + Gripper Power ON)
# When the robot operation is not accurate, use this function to reinitialize the robot.
# 
# Usage example:
#       (Remotely connect to Real RBY1-A robot UPC)
#       python3 initialize_unlimit.py
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
robot = ysw.initialize_unlimit(robotConfig.mode, robotConfig.address, robotConfig.model, option=ysw.CONNECT_AND_POWER)