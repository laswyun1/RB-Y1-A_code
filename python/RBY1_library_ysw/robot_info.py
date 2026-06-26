# Constant Robot Information File
# This should be imported only in each library files
# 
# Usage example:
#       import robot_info as RI
#
# Copyright (c) 2026 Precision Mechatronics Laboratory, KAIST. All rights reserved.
# Author: Seungwon Yun

import numpy as np


# =============================================================================
# Simple Calculation Helper
# =============================================================================
TWO_PI = 2.0 * np.pi
RAD2DEG = 180.0 / np.pi
DEG2RAD = np.pi / 180.0
RAD     = "rad"
DEG     = "deg"


# =============================================================================
# Robot Operation Mode
# =============================================================================

# 0: Docker Simulation 
# 1: Real Robot(Wifi Remote Access mode) - Cannot use peripherals
# 2: Real Robot (UPC Direct mode)

MODE_SIM = "SIM"
MODE_LOCAL_COMPUTER = "Local COM"
MODE_DIRECT_UPC = "Direct UPC"



# =============================================================================
# Robot Address
# =============================================================================

ADDRESS_SIM = "localhost:50051"
ADDRESS_UPC = "192.168.30.1:50051"
ADDRESS_RPC = "192.168.12.1:50051"



# =============================================================================
# Robot Model
# =============================================================================

MODEL_A = "a"
MODEL_M = "m"
MODEL_UB = "ub"



# =============================================================================
# Robot Initialize Option (default_lib -> "initialize" function)
# =============================================================================

CONNECT_ONLY      = 0           
CONNECT_AND_POWER = 1



# =============================================================================
# Power / Servo Target
# =============================================================================

POWER_TARGET_ALL = ".*"
SERVO_TARGET_ALL = ".*"



# =============================================================================
# Tool Flange Voltage
# =============================================================================

TOOL_FLANGE_VOLTAGE_OFF = 0
TOOL_FLANGE_VOLTAGE_ON = 12



# =============================================================================
# LED Color
# =============================================================================

LED_RED    = (255, 0,   0)
LED_ORANGE = (255, 127, 0)
LED_YELLOW = (255, 255, 0)
LED_GREEN  = (0,   255, 0)
LED_BLUE   = (0,   0,   255)
LED_INDIGO = (75,  0,   130)
LED_VIOLET = (148, 0,   211)
LED_WHITE  = (255, 255, 255)



# =============================================================================
# 22-DoF Joint Position for (Head + Body) Postures
# =============================================================================

ZERO_POS = {"torso":     np.deg2rad([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "right_arm": np.deg2rad([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "left_arm":  np.deg2rad([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "head":      np.deg2rad([0.0, 0.0])}

READY_POS = {"torso":     np.deg2rad([0.0, 45.0, -90.0,   45.0, 0.0,  0.0]),
             "right_arm": np.deg2rad([0.0, -5.0,   0.0, -120.0, 0.0, 70.0, 0.0]),
             "left_arm":  np.deg2rad([0.0,  5.0,   0.0, -120.0, 0.0, 70.0, 0.0]),
             "head":      np.deg2rad([0.0, 0.0])}

PACK_POS = {"torso":     np.deg2rad([0.0, 80.0, -140.0,   60.0, 0.0,   0.0]),
            "right_arm": np.deg2rad([0.0,  0.0,    0.0, -150.0, 0.0, 105.0, 0.0]),
            "left_arm":  np.deg2rad([0.0,  0.0,    0.0, -150.0, 0.0, 105.0, 0.0]),
            "head":      np.deg2rad([0.0, 0.0])}

READY_DEMO_POS = {"torso":     np.deg2rad([0.0, 45.0, -90.0,   45.0, 0.0,  0.0]),
                  "right_arm": np.deg2rad([0.0, -5.0,   0.0, -130.0, 0.0, 60.0, 90.0]),
                  "left_arm":  np.deg2rad([0.0,  5.0,   0.0, -130.0, 0.0, 60.0, -90.0]),
                  "head":      np.deg2rad([0.0, 30.0])}



# =============================================================================
# 2-DoF Joint Position for (Head) Postures [0: Yaw, 1: Pitch]
# =============================================================================

HEAD_POS_DEMO = [0.0, np.deg2rad(30.0)] 
HEAD_ZERO_POS = [0.0, 0.0]



# =============================================================================
# Grippers (Dynamixel XM430-W210) 
# =============================================================================

GRIPPER_DEVICE_NAME                     = "/dev/rby1_gripper"
GRIPPER_ID_R                            = 0
GRIPPER_ID_L                            = 1

DXL_PROTOCOL_VERSION                    = 2.0
DXL_BAUDRATE                            = 2000000

DXL_ADDR_OPERATING_MODE                 = 11      # 0: Current Control / 1: Velocity Control / 3: Position Control(default)
DXL_ADDR_CURRENT_LIMIT                  = 38      # 0 ~ 1193
DXL_ADDR_TORQUE_ENABLE                  = 64      # 0: Turn OFF(default) / 1: Turn ON
DXL_ADDR_LED                            = 65      # 0: Turn OFF(default) / 1: Turn ON
DXL_ADDR_GOAL_CURRENT                   = 102
DXL_ADDR_GOAL_POSITION                  = 116   
DXL_ADDR_PRESENT_CURRENT                = 126     # Unit: 2.69mA
DXL_ADDR_PRESENT_POSITION               = 132     # Unit: 1-pulse(or Tick)

DXL_CURRENT_CONTROL_MODE                = 0
DXL_POSITION_CONTROL_MODE               = 3 
DXL_CURRENT_BASED_POSITION_CONTROL_MODE = 5
DXL_CONTROL_MODE                        = [DXL_CURRENT_CONTROL_MODE, 
                                           DXL_POSITION_CONTROL_MODE, 
                                           DXL_CURRENT_BASED_POSITION_CONTROL_MODE]

DXL_TORQUE_DISABLE                      = 0
DXL_TORQUE_ENABLE                       = 1

DXL_MIN_POS_VALUE                       = 0
DXL_MAX_POS_VALUE                       = 4095
DXL_RESOLUTION_TICKS_PER_REV            = 4096.0
DXL_AMPERE_PER_UNIT                     = 0.00269    # 1-unit = 2.69mA

DXL_CURRENT_LIMIT_MAX_A                 = 3.2               # Unit: A
DXL_CURRENT_LIMIT_MAX_RAW               = 1193              # Unit: 2.69mA

DXL_COMM_SUCCESS                        = 0
DXL_NO_ERROR                            = 0



DEMO_GRIP_CURRENT_R             = 0.8
DEMO_GRIP_CURRENT_L             = 0.6
DEMO_LAY_CURRENT_R              = -0.04
DEMO_LAY_CURRENT_L              = -0.03
DEMO_ZEROING_CURRENT_L          = 0.03
DEMO_LAY_CURRENT_END            = 0.0



