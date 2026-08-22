# Move forward/backward by using Mobile Wheels
# 
# Usage example:
#       (Remotely connect to Real RBY1-A robot UPC)
#       Before use this code, Robot should be powered ON
#       python3 mobile_linear.py
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

print("[Mobile: Linear Move] Starts")

try:
    while True:
        try:
            # Select Desired Distance
            distance_cm = float(input(f"Enter the desired distance[cm] (+)Forward, (-)Backward: "))
        except ValueError:
            print("[ERROR] Invalid input. Please enter a number.")
            continue
        
        distance_m = distance_cm * 0.01
    
        ysw.mobile_linear_distance(
            robot,
            distance_m=distance_m,
            max_speed=0.25,
            accel=0.8,
            tolerance_m=0.01
        )
        
        print("\n\n")
        
except KeyboardInterrupt:
    print("\n[Mobile: Linear Move] Ends")

finally:
    pass
