# Turn CCW/CW by using Mobile Wheels
# 
# Usage example:
#       (Remotely connect to Real RBY1-A robot UPC)
#       Before use this code, Robot should be powered ON
#       python3 mobile_angular.py
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

print("[Mobile: Angular Turn] Starts")

try:
    while True:
        try:
            # Select Desired Distance
            angle_deg = float(input(f"Enter the desired angle[deg] (+)CCW, (-)CW: "))
        except ValueError:
            print("[ERROR] Invalid input. Please enter a number.")
            continue
           
        ysw.mobile_turn_inplace(
            robot,
            angle_deg=angle_deg,
            max_yaw_vel=0.3,
            yaw_accel=1.5,
            tolerance_deg=1.0
        )
        
        print("\n\n")
        
except KeyboardInterrupt:
    print("\n[Mobile: Angular Turn] Ends")

finally:
    pass
