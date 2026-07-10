# Impedance Control Test
# Default: reference link is "base"
# You can change the reference/target links and 
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


## Need Caution for torso GC !!
torso_IC_used = ysw.check_yN(
    "Turn ON the Impedance Control of 'torso' [y/N]: "
)
right_arm_IC_used = ysw.check_yN(
    "Turn ON the Impedance Control of 'right_arm' [y/N]: "
)
left_arm_IC_used = ysw.check_yN(
    "Turn ON the Impedance Control of 'left_arm' [y/N]: "
)

control_hold_time_used = float(input("Enter the 'control_hold_time': "))

if not torso_IC_used and not right_arm_IC_used and not left_arm_IC_used:
    print("No component is selected. Impedance Control is not started.")
else:
    ### Get current pos and rpy
    HT_torso = ysw.get_FK(robot, reference_link="base", target_link="link_torso_5")
    HT_right = ysw.get_FK(robot, reference_link="base", target_link="ee_right")
    HT_left  = ysw.get_FK(robot, reference_link="base", target_link="ee_left")

    pos_torso = HT_torso['position']
    pos_right = HT_right['position']
    pos_left  = HT_left['position']
    rpy_torso = HT_torso['rpy']
    rpy_right = HT_right['rpy']
    rpy_left  = HT_left['rpy']

    ### Impedance Position Control example (Component-wise version)
    impedance_targets_sample = [    
        {
            "moving_component": "torso",
            "reference_link": "base",
            "target_link": "link_torso_5",
            "pos": pos_torso,                   
            "rpy": rpy_torso,
            "translation_weight": [1500, 1500, 1500],
            "rotation_weight": [100, 100, 100],
            "damping_ratio": 0.2
        },
        {
            "moving_component": "right_arm",
            "reference_link": "base",
            "target_link": "ee_right",
            "pos": pos_right,          
            "rpy": rpy_right,
            "translation_weight": [500, 500, 500],
            "rotation_weight": [50, 50, 50],
            "damping_ratio": 0.7,
        },  
        {
            "moving_component": "left_arm",
            "reference_link": "base",
            "target_link": "ee_left",
            "pos": pos_left,          
            "rpy": rpy_left,
            "translation_weight": [500, 500, 500],
            "rotation_weight": [50, 50, 50],
            "damping_ratio": 0.7,
        },                      
    ]

    handler = None
    try:
        handler = ysw.impedanceCtrl_cartesian_componentWise(
            robot, 
            impedance_targets=impedance_targets_sample, 
            control_hold_time=control_hold_time_used,
            get_return=False
        )
        print("Impedance Controlis running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("Impedance Control stopped by USER.")

    finally:
        print("[Impedance Control] Ends")