# Right/Left Gripper Motions by Current-based Position Control
# This function adds released holding current for small position error to "gripper_posCtrl.py"
# 
# Usage example:
#       (Remotely connect to Real RBY1-A robot UPC)
#       Before use this code, Robot should be powered ON
#       python3 gripper_posCtrl_smooth.py
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
import threading



# =============================================================================
# Shared variables
# =============================================================================
target_position = None
target_version = 0

target_lock = threading.Lock()
stop_event = threading.Event()


def gripper_control_loop(gripper):
    move_current = 0.02
    hold_current = 0.005

    control_period = 0.05

    last_target_version = -1
    mode_R = "IDLE"
    mode_L = "IDLE"

    while not stop_event.is_set():
        with target_lock:
            target = target_position
            version = target_version

        if target is None:
            time.sleep(control_period)
            continue

        # New target is given: send move_current command once
        if version != last_target_version:
            gripper.current_based_position_control_linear(ysw.GRIPPER_ID_R, current_ampere_mag=move_current, linear_pos=target)
            gripper.current_based_position_control_linear(ysw.GRIPPER_ID_L, current_ampere_mag=move_current, linear_pos=target)
            mode_R = "MOVING"
            mode_L = "MOVING"
            last_target_version = version

        pos_R = gripper.read_position(ysw.GRIPPER_ID_R, unit=ysw.MM, homing_use=True, print_log=False)
        pos_L = gripper.read_position(ysw.GRIPPER_ID_L, unit=ysw.MM, homing_use=True, print_log=False)
        err_R = abs(target - pos_R)
        err_L = abs(target - pos_L)

        # After nearly close to target position,  send hold_current command once
        if mode_R == "MOVING" and err_R < ysw.GRIPPER_POS_ERROR_TOLERANCE:
            gripper.current_based_position_control_linear(ysw.GRIPPER_ID_R, current_ampere_mag=hold_current, linear_pos=target)
            mode_R = "HOLDING"

        if mode_L == "MOVING" and err_L < ysw.GRIPPER_POS_ERROR_TOLERANCE:
            gripper.current_based_position_control_linear(ysw.GRIPPER_ID_L, current_ampere_mag=hold_current, linear_pos=target)
            mode_L = "HOLDING"

        # During "HOLDING", if error becomes larger, send move_current command once
        if mode_R == "HOLDING" and err_R > ysw.GRIPPER_POS_ERROR_TOLERANCE:
            gripper.current_based_position_control_linear(ysw.GRIPPER_ID_R, current_ampere_mag=move_current, linear_pos=target)
            mode_R = "MOVING"

        if mode_L == "HOLDING" and err_L > ysw.GRIPPER_POS_ERROR_TOLERANCE:
            gripper.current_based_position_control_linear(ysw.GRIPPER_ID_L, current_ampere_mag=move_current, linear_pos=target)
            mode_L = "MOVING"

        time.sleep(control_period)


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
    
    

control_thread = threading.Thread(
    target=gripper_control_loop,
    args=(gripper,),
    daemon=True
)
control_thread.start()

### Input desired positions(0mm - 91mm)
try:
    while True:
        try:
            new_target_pos = float(input("Enter the Desired Position[mm] (0~91): "))
        except ValueError:
            print("Please enter a numeric value.")
            continue

        if new_target_pos < 0.0 or new_target_pos > 91.0:
            print("Desired Position should be in [0 mm, 91 mm]")
            continue

        with target_lock:
            target_position = new_target_pos
            target_version += 1
            
        time.sleep(3)       # You can erase this if you do not need to see "input" explanation

except KeyboardInterrupt:
    pass

finally:
    stop_event.set()
    control_thread.join(timeout=1.0)

    gripper.disable(ysw.GRIPPER_ID_R)
    gripper.disable(ysw.GRIPPER_ID_L)

    print("[Gripper Current-based Position Control] Ends")    