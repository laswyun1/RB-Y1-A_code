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
TICK    = "tick"
MM      = "mm"
POSITION_UNIT = [RAD, DEG, TICK, MM]


# =============================================================================
# Robot Operation Mode
# =============================================================================

# 0: Docker Simulation 
# 1: Real Robot(Wifi Remote Access mode) - Cannot use peripherals
# 2: Real Robot (UPC Direct mode)

MODE_SIM = "SIM"
MODE_LOCAL_COMPUTER = "Local COM"
MODE_LOCAL_COMPUTER_PML_WIFI = "PML"
MODE_DIRECT_UPC = "Direct UPC"



# =============================================================================
# Robot Address
# =============================================================================

ADDRESS_SIM = "localhost:50051"
ADDRESS_UPC = "192.168.30.1:50051"
ADDRESS_RPC = "192.168.12.1:50051"

ADDRESS_PML_WIFI = "192.168.1.217:50051"    # (Can be changed from UPC ipv4 setting)



# =============================================================================
# Robot Model
# =============================================================================

MODEL_A = "a"
MODEL_M = "m"
MODEL_UB = "ub"



# =============================================================================
# Degree of Freedom (DoF)
# =============================================================================

DOF_HEAD        = 2
DOF_WHEEL       = 2
DOF_TORSO       = 6
DOF_RIGHT_ARM   = 7
DOF_LEFT_ARM    = 7
DOF_BODY        = 20    # Torso + Arms
DOF_TOTAL       = 24



# =============================================================================
# Components Name
# =============================================================================

WHEEL                    = 'wheel'
TORSO                    = "torso"
RIGHT_ARM                = "right_arm"
LEFT_ARM                 = "left_arm"
HEAD                     = "head"

BODY_COMPONENT_NAME_LIST = [TORSO, RIGHT_ARM, LEFT_ARM]
ALL_COMPONENT_NAME_LIST  = [WHEEL, TORSO, RIGHT_ARM, LEFT_ARM, HEAD]



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
# Link Name
# =============================================================================

ALL_LINK_NAMES = ["base",
                  "wheel_r",
                  "wheel_l",
                  "link_torso_0",
                  "link_torso_1",
                  "link_torso_2",
                  "link_torso_3",
                  "link_torso_4",
                  "link_torso_5",
                  "link_right_arm_0",
                  "link_right_arm_1",
                  "link_right_arm_2",
                  "link_right_arm_3",
                  "link_right_arm_4",
                  "link_right_arm_5",
                  "link_right_arm_6",
                  "link_left_arm_0",
                  "link_left_arm_1",
                  "link_left_arm_2",
                  "link_left_arm_3",
                  "link_left_arm_4",
                  "link_left_arm_5",
                  "link_left_arm_6",
                  "ee_right",
                  "ee_finger_r1",
                  "ee_finger_r2",
                  "ee_left",
                  "ee_finger_l1",
                  "ee_finger_l2",
                  "link_head_0",
                  "link_head_1",
                  "link_head_2"]

TOTAL_LINK_NUMBER = 32



# =============================================================================
# Joint Name
# =============================================================================

ALL_JOINT_NAMES = ["right_wheel",
                   "left_wheel",
                   "torso_0",
                   "torso_1",
                   "torso_2",
                   "torso_3",
                   "torso_4",
                   "torso_5",
                   "right_arm_0",
                   "right_arm_1",
                   "right_arm_2",
                   "right_arm_3",
                   "right_arm_4",
                   "right_arm_5",
                   "right_arm_6",
                   "left_arm_0",
                   "left_arm_1",
                   "left_arm_2",
                   "left_arm_3",
                   "left_arm_4",
                   "left_arm_5",
                   "left_arm_6",
                   "head_0",
                   "head_1",]

TOTAL_JOINT_NUMBER = 24



# =============================================================================
# Joint Index
# =============================================================================

ALL_JOINT_INDEX = {"right_wheel": 0,
                   "left_wheel": 1,
                   "torso_0": 2,
                   "torso_1": 3,
                   "torso_2": 4,
                   "torso_3": 5,
                   "torso_4": 6,
                   "torso_5": 7,
                   "right_arm_0": 8,
                   "right_arm_1": 9,
                   "right_arm_2": 10,
                   "right_arm_3": 11,
                   "right_arm_4": 12,
                   "right_arm_5": 13,
                   "right_arm_6": 14,
                   "left_arm_0": 15,
                   "left_arm_1": 16,
                   "left_arm_2": 17,
                   "left_arm_3": 18,
                   "left_arm_4": 19,
                   "left_arm_5": 20,
                   "left_arm_6": 21,
                   "head_0": 22,
                   "head_1": 23}



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

GRIPPER_DEBUG_POS = {"torso":     np.deg2rad([0.0, 80.0, -140.0,   60.0, 0.0,   0.0]),
                     "right_arm": np.deg2rad([0.0,  0.0,    0.0, -150.0, 0.0, -20.0, 0.0]),
                     "left_arm":  np.deg2rad([0.0,  0.0,    0.0, -150.0, 0.0, -20.0, 0.0]),
                     "head":      np.deg2rad([0.0, 0.0])}

# =============================================================================
# [260723] DEMO-specific 22-DoF Joint Position for (Head + Body) Postures 
# =============================================================================
# 처음 테이블을 바라보고 있는 상태(시작 상태)
DEMO_BASIC_POS_260723 = {"torso":     np.deg2rad([0.0, 30.0, -60.0,  30.0, 0.0,  0.0]),
                         "right_arm": np.deg2rad([0.0, -5.0,   0.0, -140.0, 0.0, 100.0, 0.0]),
                         "left_arm":  np.deg2rad([0.0,  5.0,   0.0, -140.0, 0.0, 100.0, 0.0]),
                         "head":      np.deg2rad([0.0, 0.0])}

# 선반 위의 물건을 집기 위해 선반쪽으로 몸을 돌리고, 키를 높인 상태
DEMO_TURN_POS_260723 = {"torso":     np.deg2rad([0.0,   0.0,  0.0,    0.0, 0.0,  90.0]),
                        "right_arm": np.deg2rad([0.0, -10.0,   0.0, -145.0, 0.0, 80.0, 0.0]),
                        "left_arm":  np.deg2rad([0.0,  10.0,   0.0, -145.0, 0.0, 80.0, 0.0]),
                        "head":      np.deg2rad([0.0, 20.0])}

# 물체 잡고 나서, 다시 로봇 쪽으로 끌어오는 동작
DEMO_AFTER_GRASP_POS_260723 = {"torso":     np.deg2rad([0.0,   0.0,  0.0,    0.0, 0.0,  90.0]),
                               "right_arm": np.deg2rad([0.0, -10.0,   0.0, -150.0, 0.0, 80.0, 0.0]),
                               "left_arm":  np.deg2rad([0.0,  10.0,   0.0, -150.0, 0.0, 80.0, 0.0]),
                               "head":      np.deg2rad([0.0, 20.0])}

# 테이블 위 물체를 팔가림없이 비전이 볼 수 있는 상태
DEMO_VISION_BASIC_POS_260723 = {"torso":     np.deg2rad([0.0, 30.0, -60.0,   30.0, 0.0,  0.0]),
                                "right_arm": np.deg2rad([0.0, -30.0,   0.0, -120.0, 0.0, 70.0, 0.0]),
                                "left_arm":  np.deg2rad([0.0,  30.0,   0.0, -120.0, 0.0, 70.0, 0.0]),
                                "head":      np.deg2rad([0.0, 40.0])}

# 왼손에 집은 물체를 오른손으로 옮기는 자세 (manual, 이 자세 전에 오른손 그리퍼가 열려있어야 함)
DEMO_LEFT_TO_RIGHT_POS_BEFORE_260723 = {"torso":    np.deg2rad([0.0, 30.0, -60.0,  30.0, 0.0,  0.0]),
                                        "right_arm": np.array([-0.1225, -0.4689, -0.0202, -2.2191, -1.644,   1.1328,  0.2777]),
                                        "left_arm":  np.array([-0.3439,  0.4972,  0.004,  -2.2403, 1.4097,  1.2995, -0.1164]),
                                        "head":      np.deg2rad([0.0, 40.0])}

DEMO_LEFT_TO_RIGHT_POS_AFTER_260723 = {"torso":    np.deg2rad([0.0, 30.0, -60.0,  30.0, 0.0,  0.0]),
                                       "right_arm": np.array([-0.2732, -0.3805,  0.1492, -2.176,  -1.5426,  1.0992,  0.2774]),
                                       "left_arm":  np.array([-0.3439,  0.4972,  0.004,  -2.2403, 1.4097,  1.2995, -0.1164]),
                                       "head":      np.deg2rad([0.0, 40.0])}

DEMO_RIGHT_TO_LEFT_POS_BEFORE_260723 = {"torso":    np.deg2rad([0.0, 30.0, -60.0,  30.0, 0.0,  0.0]),
                                        "right_arm": np.array([-0.3243, -0.5258,  0.1003, -2.1467, -1.4445,  1.1435,  0.1834]),
                                        "left_arm":  np.array([-0.3439,  0.4972,  0.004,  -2.2403,  1.4097,  1.2995, -0.1164]),
                                        "head":      np.deg2rad([0.0, 40.0])}

DEMO_RIGHT_TO_LEFT_POS_AFTER_260723 = {"torso":    np.deg2rad([0.0, 30.0, -60.0,  30.0, 0.0,  0.0]),
                                       "right_arm": np.array([-0.3243, -0.5258,  0.1003, -2.1467, -1.4445,  1.1435,  0.1834]),
                                       "left_arm":  np.array([-0.332,   0.3887, -0.0811, -2.2015,  1.43,    1.2327, -0.04]),
                                       "head":      np.deg2rad([0.0, 40.0])}


# =============================================================================
# [260826] DEMO-specific 22-DoF Joint Position for (Head + Body) Postures 
# =============================================================================
# 처음 테이블을 바라보고 있는 상태(시작 상태)
DEMO_BASIC_POS_260826 = {"torso":     np.deg2rad([0.0, 25.0, -50.0,  25.0, 0.0,  0.0]),
                         "right_arm": np.deg2rad([0.0, -28.0,  0.0, -140.0, 0.0, 100.0, 90.0]),
                         "left_arm":  np.deg2rad([0.0,  28.0,  0.0, -140.0, 0.0, 100.0, -90.0]),
                         "head":      np.deg2rad([0.0, 43.0])}




# =============================================================================
# 2-DoF Joint Position for (Head) Postures [0: Yaw, 1: Pitch]
# =============================================================================

HEAD_DEMO_POS = [0.0, np.deg2rad(30.0)] 
HEAD_ZERO_POS = [0.0, 0.0]



# =============================================================================
# Parameters for Cartesian Control
# =============================================================================

LINEAR_VELOCITY_LIMIT       = 1.5
ANGULAR_VELOCITY_LIMIT      = 1.5
ACCELERATION_LIMIT          = 1.0

MINIMUM_TIME                        = 4
STOP_ORIENTATION_TRACKING_ERROR     = 0.01
STOP_POSITION_TRACKING_ERROR        = 0.01



# =============================================================================
# Parameters for (Cartesian) Impedance Control
# =============================================================================

DEFAULT_TRANSLATION_WEIGHT      = [1000, 1000, 1000]
DEFAULT_ROTATION_WEIGHT         = [100, 100, 100]
DEFAULT_DAMPING_RATIO           = 0.85
DEFAULT_CONTROL_HOLD_TIME       = 2



# =============================================================================
# Parameters for (Joint) Impedance Control
# =============================================================================

DEFAULT_STIFFNESS       = 100.0
DEFAULT_TORQUE_LIMIT    = 10.0



# =============================================================================
# Parameters for Optimal Control
# =============================================================================

WEIGHT              = 1
STOP_COST           = WEIGHT * WEIGHT * 2e-3
MIN_DELTA_COST      = WEIGHT * WEIGHT * 2e-3
PATIENCE            = 10



# =============================================================================
# Parameters for Gravity Compensation
# =============================================================================

GC_SET_ON   = True
GC_SET_OFF  = False



# =============================================================================
# Grippers (Dynamixel XM430-W210) 
# =============================================================================

GRIPPER_DEVICE_NAME                     = "/dev/rby1_gripper"
GRIPPER_ID_R                            = 0
GRIPPER_ID_L                            = 1

# For Homing  
# Set "Open" as 0-tick and "Closed" has maximum (negative) tick values
GRIPPER_HOMING_OPEN_CURRENT             = -0.02                 # Unit: A
GRIPPER_HOMING_CLOSE_CURRENT            = 0.02                  # Unit: A
GRIPPER_OPEN_POS_RAWTICK_R              = -1965                 # Unit: 32-bit Position Tick    # Raw position tick value when gripper is fully opened 
GRIPPER_OPEN_POS_RAWTICK_L              = -925                  # Unit: 32-bit Position Tick    # Raw position tick value when gripper is fully opened 
GRIPPER_CLOSE_POS_RAWTICK_R             = 899                   # Unit: 32-bit Position Tick    # Raw position tick value when gripper is fully closed (= OFFSET / Set 0-tick)
GRIPPER_CLOSE_POS_RAWTICK_L             = 2015                  # Unit: 32-bit Position Tick    # Raw position tick value when gripper is fully closed (= OFFSET / Set 0-tick)
GRIPPER_POS_RANGE_TICK_R                = -2864                 # Unit: 32-bit Position Tick    # GRIPPER_OPEN_POS_RAWTICK_R - GRIPPER_CLOSE_POS_RAWTICK_R
GRIPPER_POS_RANGE_TICK_L                = -2940                 # Unit: 32-bit Position Tick    # GRIPPER_OPEN_POS_RAWTICK_L - GRIPPER_CLOSE_POS_RAWTICK_L
GRIPPER_POS_RANGE_LINEAR                = 91.0                  # Unit: mm      # Measured by vernier caliper (physically cushion to cushion)
GRIPPER_POS_SCALE_LINEAR_TO_TICK_R      = int(GRIPPER_POS_RANGE_TICK_R / 91.0)     # Unit: tick/mm
GRIPPER_POS_SCALE_LINEAR_TO_TICK_L      = int(GRIPPER_POS_RANGE_TICK_L / 91.0)     # Unit: tick/mm

GRIPPER_POS_ERROR_TOLERANCE             = 1       # Unit: mm  (Manually set)
 
DXL_PROTOCOL_VERSION                    = 2.0
DXL_BAUDRATE                            = 2000000

DXL_ADDR_OPERATING_MODE                 = 11      # 0: Current Control / 1: Velocity Control / 3: Position Control(default)
DXL_ADDR_CURRENT_LIMIT                  = 38      # 0 ~ 1193
DXL_ADDR_TORQUE_ENABLE                  = 64      # 0: Turn OFF(default) / 1: Turn ON
DXL_ADDR_LED                            = 65      # 0: Turn OFF(default) / 1: Turn ON
DXL_ADDR_D_GAIN_POSCTRL                 = 80      # default: 0      range: 0~16,383
DXL_ADDR_I_GAIN_POSCTRL                 = 82      # default: 0      range: 0~16,383
DXL_ADDR_P_GAIN_POSCTRL                 = 84      # default: 800    range: 0~16,383
DXL_ADDR_GOAL_CURRENT                   = 102
DXL_ADDR_GOAL_POSITION                  = 116   
DXL_ADDR_PRESENT_CURRENT                = 126     # Unit: 2.69mA
DXL_ADDR_PRESENT_POSITION               = 132     # Unit: 1-pulse(or Tick)

DXL_CURRENT_CONTROL_MODE                = 0
DXL_POSITION_CONTROL_MODE               = 3       # It only used for (0~4096) positive tick values (Not suited for RBY1)
DXL_EXTENDED_POSITION_CONTROL_MODE      = 4       # It is NOT suited for RBY1 gripper position control (Current Limit value is NOT ensured)
DXL_CURRENT_BASED_POSITION_CONTROL_MODE = 5       # It is SUITED for RBY1 Gripper Position Control (raw gripper tick: [about +900(Close)] ~ [about -1900(Open)])
DXL_CONTROL_MODE                        = [DXL_CURRENT_CONTROL_MODE, 
                                           DXL_POSITION_CONTROL_MODE, 
                                           DXL_EXTENDED_POSITION_CONTROL_MODE,  
                                           DXL_CURRENT_BASED_POSITION_CONTROL_MODE]

DXL_TORQUE_DISABLE                      = 0
DXL_TORQUE_ENABLE                       = 1

DXL_MIN_POS_VALUE                       = 0
DXL_MAX_POS_VALUE                       = 4095
DXL_RESOLUTION_TICKS_PER_REV            = 4096.0
DXL_AMPERE_PER_UNIT                     = 0.00269    # 1-unit = 2.69mA

DXL_CURRENT_LIMIT_MAX_A_DEFAULT         = 3.2               # Unit: A           (Never USE this)
DXL_CURRENT_LIMIT_MAX_RAW_DEFAULT       = 1193              # Unit: 2.69mA      (Never USE this)
DXL_CURRENT_LIMIT_MAX_A_RAINBOW         = 0.6725            # Unit: A           (Maximum USE this)
DXL_CURRENT_LIMIT_MAX_RAW_RAINBOW       = 250               # Unit: 2.69mA      (Maximum USE this)
DXL_CURRENT_LIMIT_MAX_A_YSW             = 0.1345            # Unit: A           (USE this)           
DXL_CURRENT_LIMIT_MAX_RAW_YSW           = 50                # Unit: 2.69mA      (Use this)

DXL_COMM_SUCCESS                        = 0
DXL_NO_ERROR                            = 0

DXL_P_GAIN_POSCTRL_DEFAULT              = 800           # By RainbowRobotics
DXL_I_GAIN_POSCTRL_DEFAULT              = 0             # By RainbowRobotics
DXL_D_GAIN_POSCTRL_DEFAULT              = 4000          # By RainbowRobotics

DXL_P_GAIN_POSCTRL                      = 500
DXL_I_GAIN_POSCTRL                      = 0
DXL_D_GAIN_POSCTRL                      = 2000
DXL_PID_GAIN_LIMIT                      = 5000          # Manually setting for hazard (you can change this)

DEMO_GRIP_CURRENT_R             = 0.8
DEMO_GRIP_CURRENT_L             = 0.6
DEMO_LAY_CURRENT_R              = -0.04
DEMO_LAY_CURRENT_L              = -0.03
DEMO_ZEROING_CURRENT_L          = 0.03
DEMO_LAY_CURRENT_END            = 0.0

GRIPPER_Z_OFFSET_FROM_LINK_EE_FINGER = -0.0605      # Unit=[m], Fingertip offset from "link_ee_finger_r1" or "link_ee_finger_l1" frames (using EE_FINGER.dae)
GRIPPER_OFFSET_BEFORE_GRASP          = 0.03         # Unit=[m], Offset before gripper grasp (or open) | It should be applied after applying "max(3D_size(object)/2)" offset
GRIPPER_SAFETY_OPEN_OFFSET           = 10.0         # Unit=[mm], Safety margin for position control of gripper (open)
GRIPPER_SAFETY_CLOSE_OFFSET          = -1.2*GRIPPER_SAFETY_OPEN_OFFSET        # Unit=[mm], for position control of gripper (close)



# =============================================================================
# IntelRealsense Depth Camera Info 
# =============================================================================
HEAD_D435I_SERIAL = "250122070658"
RIGHT_D405_SERIAL  = "260322279815"
LEFT_D405_SERIAL  = "260422275816"

SUPPORT_FRAME_ANGLE = 25        # Unit: [deg], 레인보우로보틱스에서 받은 손목카메라 프레임의 휘어진 각도


# =============================================================================
# Vision Colored Object Detection: HSV mask 
# (Ex): [([Lower bound 1], [upper bound 1]), ([Lower bound 2], [upper bound 2]), ...]
# =============================================================================

HSV_MASK_RED      = [(np.array([0, 80, 80]), np.array([10, 255, 255])),       
                     (np.array([170, 80, 80]), np.array([180, 255, 255]))]
HSV_MASK_RED_BOX  = [(np.array([0, 128, 98]), np.array([4, 255, 255])),
                     (np.array([176, 128, 98]), np.array([179, 255, 255]))]
# HSV_MASK_BLACK  = [(np.array([0, 0, 0]), np.array([179, 255, 30]))]       # Strict
HSV_MASK_BLACK    = [(np.array([0, 0, 0]), np.array([179, 255, 60]))]         # Not strict
HSV_MASK_BLACK_BOX  = [(np.array([0, 0, 0]), np.array([179, 255, 70]))]     # Not strict
HSV_MASK_SILVER = [(np.array([0, 0, 80]), np.array([180, 60, 255]))]
# HSV_MASK_YELLOW = [(np.array([15, 40, 40]), np.array([45, 255, 255]))]
HSV_MASK_YELLOW = [(np.array([10, 100, 80]), np.array([35, 255, 255]))]

HSV_MASK_ORANGE = [(np.array([4, 80, 60]), np.array([24, 255, 255]))]
HSV_MASK_GREEN  = [(np.array([35, 60, 60]), np.array([85, 255, 255]))]


# =============================================================================
# D435i Parameters
# =============================================================================

# Should be strictly measured!!!
D435I_Z_OFFSET_FROM_LINK_HEAD_2      = 0.07      # Unit=[m]
D435I_X_OFFSET_FROM_D435I_CENTER     = 0.0165     # Unit=[m], (Official: 0.0175m), Origin standard: Left image camera | offset of D435i origin frame(measure) from the hardware center 
D435I_DEPTH_Z_OFFSET_FROM_LENS_PLANE = -0.01     # Unit=[m], Reference=Lens plane | Depth(z) standard point position offset relative to D435i lens plane



