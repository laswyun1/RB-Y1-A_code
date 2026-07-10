# Impedance Control Test
# 
# Usage example:
#       (Remotely connect to Real RBY1-A robot UPC)
#       Before use this code, Robot should be powered ON
#       Before use this code, please use "pose_.py" file to move the robot in initial posture
#       python3 control_impedance.py
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
# MODE_DIRECT_UPC
robotConfig = ysw.setRobotConfig(ysw.MODE_DIRECT_UPC, ysw.MODEL_A)
robot = ysw.initialize(robotConfig.mode, robotConfig.address, robotConfig.model, option=ysw.CONNECT_ONLY)


print("[Impedance Control] Starts")


torso_GC_used = False
## Need Caution for torso GC !!
# torso_GC_used = ysw.check_yN(
#     "Turn ON the Gravity Compensation of 'torso' [y/N]: "
# )
right_arm_GC_used = ysw.check_yN(
    "Turn ON the Gravity Compensation of 'right_arm' [y/N]: "
)
left_arm_GC_used = ysw.check_yN(
    "Turn ON the Gravity Compensation of 'left_arm' [y/N]: "
)

control_hold_time_used = float(input("Enter the 'control_hold_time': "))


if not torso_GC_used and not right_arm_GC_used and not left_arm_GC_used:
    print("No component is selected. Gravity compensation is not started.")
else:
    handler = None
    try:
        handler = ysw.gravityCompensation(
            robot,
            GC_ONOFF=ysw.GC_SET_ON,
            torso=torso_GC_used,
            right_arm=right_arm_GC_used,
            left_arm=left_arm_GC_used,
            control_hold_time=control_hold_time_used,
            get_return=False
        )
        print("Gravity compensation is running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("Gravity compensation stopped by USER.")

    finally:
        ysw.gravityCompensation(
            robot,
            GC_ONOFF=ysw.GC_SET_OFF,
            torso=torso_GC_used,
            right_arm=right_arm_GC_used,
            left_arm=left_arm_GC_used,
            control_hold_time=1.0,
            get_return=True
        )
        print("[Impedance Control] Ends")