# Right/Left Gripper PID gain Tuning
#
# This is only for operation mode 5: current-based position control mode
# 
# Usage example:
#       (Remotely connect to Real RBY1-A robot UPC)
#       Before use this code, Robot should be powered ON
#       python3 gripper_PIDgainTuning.py
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

print("[Gripper PID gaing Tuning for (Current-based position control mode)] Starts")

gripper_R_used = ysw.check_yN("Tune the 'Right' gripper [y/N]: ")
gripper_L_used = ysw.check_yN("Tune the 'Left' gripper [y/N]: ")

if not gripper_R_used and not gripper_L_used:
    print("No component is selected. Exit the tuning program.")
    
else:
    while True:
        try:
            pos = float(input("Enter the position[mm] for control (Range: 0~91mm): "))

            if 0.0 <= pos <= 91.0:
                break
            else:
                print("Invalid position. Please enter a value between 0 and 91 mm.")

        except ValueError:
            print("Invalid input. Please enter a number.")
            
    while True:
        try:
            current_mag = float(input("Enter the current limit[A] for control (Range: 0~0.3A): "))

            if 0.0 <= current_mag <= 0.3:
                break
            else:
                print("Invalid current. Please enter a value between 0 and 0.3 A.")

        except ValueError:
            print("Invalid input. Please enter a number.")

    # Initialize
    gripper = ysw.Gripper()
    gripper.connect(device_name=ysw.GRIPPER_DEVICE_NAME, protocol_version=ysw.DXL_PROTOCOL_VERSION, baudrate=ysw.DXL_BAUDRATE)
    gripper.test_ping()
    
    firstRun_R = 0
    firstRun_L = 0

    # PID tuning
    try:
        while True:
            if gripper_R_used:
                if firstRun_R == 0:
                    gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE)
                    firstRun_R = 1
                gripper.disable(ysw.GRIPPER_ID_R)
                
            if gripper_L_used:
                if firstRun_L == 0:
                    gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_L, operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE)
                    firstRun_L = 1
                gripper.disable(ysw.GRIPPER_ID_L)
            
            Kp = int(input("Enter the P gain: "))
            Ki = int(input("Enter the I gain: "))
            Kd = int(input("Enter the D gain: "))
            
            if Kp < 0 or Ki < 0 or Kd < 0:
                print("PID gains should be non-negative.")
                continue

            else:
                if gripper_R_used:
                    gripper.initialize_setting(
                        dxl_id=ysw.GRIPPER_ID_R, 
                        operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE,
                        Kp=Kp,
                        Ki=Ki,
                        Kd=Kd
                    )
                    gripper.current_based_position_control_linear(ysw.GRIPPER_ID_R, current_ampere_mag=current_mag, linear_pos=pos)
                    
                if gripper_L_used:
                    gripper.initialize_setting(
                        dxl_id=ysw.GRIPPER_ID_L, 
                        operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE,
                        Kp=Kp,
                        Ki=Ki,
                        Kd=Kd
                    )
                    gripper.current_based_position_control_linear(ysw.GRIPPER_ID_L, current_ampere_mag=current_mag, linear_pos=pos)
                
                retry = ysw.check_yN("Would you like to retry PID tuning? [y/N]: ")         # 튜닝 재수행 여부
    
                if retry:   continue
                else:       break
                    
    except KeyboardInterrupt:
        pass
    
    finally:
        gripper.disable(ysw.GRIPPER_ID_R)
        gripper.disable(ysw.GRIPPER_ID_L)
        print("\n[Gripper PID gain Tuning for (Current-based position control mode)] Ends")