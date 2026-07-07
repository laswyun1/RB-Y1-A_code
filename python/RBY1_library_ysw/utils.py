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


def check_yN(prompt: str) -> bool:
    while True:
        answer = input(prompt).strip().lower()
        if answer == "y":
            return True
        if answer == "n":
            return False
        print("Please enter 'y' or 'N'.")



# =============================================================================
# Calculations
# =============================================================================
def average_list(data):
    if not data:
        print("Empty list")
        return None
    return sum(data) / len(data)
    
    
    