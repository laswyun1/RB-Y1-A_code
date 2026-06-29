# Common Utility File
# This should be imported only in each library files
# 
# Usage example:
#       import utils as UTIL
#
# Copyright (c) 2026 Precision Mechatronics Laboratory, KAIST. All rights reserved.
# Author: Seungwon Yun

import numpy as np


# =============================================================================
# Utility Function
# =============================================================================
def print_start_title(text: str, width: int = 80, fill: str = "*"):
    msg = f" [{text}] "
    print(msg.center(width, fill) + "\n")
    
def print_end_title(text: str, width: int = 80, fill: str = "*"):
    msg = f" [{text}] "
    print("\n" + msg.center(width, fill) + "\n\n\n")

def print_start_subtitle(text: str, width: int = 80, fill: str = "="):
    msg = f" [{text}] "
    print(msg.center(width, fill) + "\n")
    
def print_end_subtitle(text: str, width: int = 80, fill: str = "="):
    msg = f" [{text}] "
    print(msg.center(width, fill) + "\n\n\n")



# =============================================================================
# Calculations
# =============================================================================
def average_list(data):
    if not data:
        print("Empty list")
        return None
    return sum(data) / len(data)
    
    
    
# =============================================================================
# Kinematics
# =============================================================================
def rpy_to_rotmat(roll, pitch, yaw):
    Rx = np.array([
        [1,      0,             0      ],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])

    Ry = np.array([
        [np.cos(pitch),  0, np.sin(pitch)],
        [       0,       1,       0      ],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [     0,           0,       1]
    ])

    return Rz @ Ry @ Rx   # xyz convention





