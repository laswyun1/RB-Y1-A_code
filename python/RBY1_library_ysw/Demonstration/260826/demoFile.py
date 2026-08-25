"""
    This is the Demo File for 260826

    [Scenario]
    (0) In 1st Floor Laboratory, the robot move forward to Table 

    (1) Black Box is in the Left side, Red Box is in the Right side on the table
        - Box location can be changed
        
    (2) 4 Black Objects(25x25x25) and 4 Red Objects(25x25x25) are located between two Boxes
        - Objects location and number can be changed

    (3) Check the Locations of "Black Box" and all "Black Objects" -> calculate IK

    (4) Pick the [Black objects] (near the center has lower index=higher priority to pick) using the Black Box located arm 
        - Ex: If Black Box is in the left side of robot, using left arm
        
    (5) Move the picking arm to Black Box and lay the Black Objects
        
    (6) Likewise, Do same works for [Red Box and Red Objects]
        
    (7) Robot grasp the [Black Box's outer side]
    
    (8) Robot grasp the [Red Box's outer side]
    
    (9) Robot goes box-transfer ready posture

    (10) The robot slightly move backward, and turn right 180 degrees

    (11) Check the shelf distance by vision camera
    
    (12) Move forward to shelf and Lay two Boxes 
"""

# ==========================================================================================================
# Import Libraries
# ==========================================================================================================
import rby1_sdk as rby
import numpy as np
import logging
import time
import pyrealsense2 as rs
import cv2

import sys
from pathlib import Path
# Path.cwd()           : folder "260826", 
# Path.cwd().parents[0]: folder "Demonstration", 
# Path.cwd().parents[1]: folder "RBY1_library_ysw"
sys.path.append(str(Path.cwd().parents[1]))                  
import rby1_lib_ysw as ysw

np.set_printoptions(suppress=True, precision=4, linewidth=300)  

# In .ipynb, show logging 
logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] %(message)s",
    # format="%(asctime)s [%(levelname)s] %(message)s",
    stream=sys.stdout,
    force=True,
)



# ==========================================================================================================
# Tuning Parameters
# ==========================================================================================================
position_control_minimum_time = 8.5      # [sec]

# Move to Table
vision_record_duration_tableDistance = 5.0    # [sec]
offset_table_distance = 0.37                  # [m]
vision_record_duration_checkBox = 5.0         # [sec]
vision_record_duration_checkObject = 5.0      # [sec]
box_minimum_area = 8000                       # [pixel^2]
box_maximum_distance = 1.2                    # [m]
object_minimum_area = 50                      # [pixel^2]
object_maximum_area = 6000                    # [pixel^2]
object_maximum_distance = 1.2                 # [m]

# minimum_time_scaler = 0.5                # For short movement
grasp_current_obj = 0.1      # [A]
object_floating_z = 0.125     # [m]
offset_gripper_open = 0.015  # [m]
manual_offset_downToGrasp = -0.008       # [m] for z-direction offset when grasp



# ==========================================================================================================
# Initialization
# ==========================================================================================================
# ----------------------------------------------------------
# Robot Initialization
# ----------------------------------------------------------
robotConfig = ysw.setRobotConfig(ysw.MODE_DIRECT_UPC, ysw.MODEL_A)          # For Real Robot + UPC
robot = ysw.initialize(robotConfig.mode, robotConfig.address, robotConfig.model, option=ysw.CONNECT_AND_POWER)
if (robot != None):
    model = robot.model()
time.sleep(1)

# ----------------------------------------------------------
# Gripper Initialization
# Gripper Setting (Need to hard check of gripper ping test)
# ----------------------------------------------------------
gripper = ysw.Gripper()
gripper.connect_hardCheck(
    device_name=ysw.GRIPPER_DEVICE_NAME, 
    protocol_version=ysw.DXL_PROTOCOL_VERSION, 
    baudrate=ysw.DXL_BAUDRATE,
    retry_interval=1.0,
    max_retry=5
)

# ----------------------------------------------------------
# Depth Camera Initialization
# D435i:  Head
# D405_R: Right Wrist
# D405_L: Left Wrist
# ----------------------------------------------------------
d435i = ysw.RealSenseDepthCamera(rgb_on=True, depth_on=True, pixel=(640,480), fps=30, serial_number=ysw.HEAD_D435I_SERIAL)
d435i.initialize()
d405_R = ysw.RealSenseDepthCamera(rgb_on=True, depth_on=True, pixel=(640,480), fps=30, serial_number=ysw.RIGHT_D405_SERIAL)
d405_R.initialize()
d405_L = ysw.RealSenseDepthCamera(rgb_on=True, depth_on=True, pixel=(640,480), fps=30, serial_number=ysw.LEFT_D405_SERIAL)
d405_L.initialize()



# ==========================================================================================================
# [Posture] Initial pose: PACK_POS
# ==========================================================================================================
desired_pos = ysw.PACK_POS  
ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time, control_hold_time=0.5)



# ==========================================================================================================
# [Vision] Check the Distance to Table
# ==========================================================================================================
# ----------------------------------------------------------
# ROI setting 
# Pixel Origin: (0,0) = 왼쪽 위 꼭짓점
# Posture: PACK_POS
# ----------------------------------------------------------
width = 640
height = 480
pixel_roi_x = [140, width - 140]      
pixel_roi_y = [320, height]    
# pixel_roi_x = None            # None이면 x 전체 사용
# pixel_roi_y = None            # None이면 y 전체 사용
    
# ----------------------------------------------------------
# Get the Distance of Center Pixel 
# Table distance is approximated as the depth of center pixel
# ----------------------------------------------------------
record_duration = vision_record_duration_tableDistance      # [sec]
distance_list = []

d435i.startPipeline()

try:
    start_time = time.time()
    
    while True:
        elapsed_time = time.time() - start_time
        
        if elapsed_time >= record_duration:
            break
        
        color_frame, depth_frame = d435i.getRawFrames()

        if color_frame is None or depth_frame is None:
            continue

        color_image, depth_image = d435i.frameToImage(color_frame, depth_frame)
        h, w, _ = color_image.shape
        
        # ROI center pixel in full image coordinate
        if pixel_roi_x is None:
            x_min = 0
            x_max = w
        else:
            x_min, x_max = pixel_roi_x
            x_min = int(np.clip(x_min, 0, w - 1))
            x_max = int(np.clip(x_max, 0, w))

        if pixel_roi_y is None:
            y_min = 0
            y_max = h
        else:
            y_min, y_max = pixel_roi_y
            y_min = int(np.clip(y_min, 0, h - 1))
            y_max = int(np.clip(y_max, 0, h))
            
        x_roi_center = (x_min + x_max) // 2
        y_roi_center = (y_min + y_max) // 2

        distance = d435i.getDepth(depth_frame, (x_roi_center, y_roi_center))   # unit = [m]
        distance_list.append(distance)
        print(f"Table Distance: {distance:.3f} [m]",)

finally:
    d435i.stopPipeline()
    d435i.destroyAllWindows()
    

# Average Distance
distance_avg = np.mean(distance_list)
print(f"\nTable Average Distance: {distance_avg} [m]")

moved_distance_to_table = distance_avg - offset_table_distance
print(f"moved_distance_to_table: {moved_distance_to_table} [m]")



# ==========================================================================================================
# [Posture] Demo Basic pose: DEMO_BASIC_POS_260826
# [Move] Move forward to Table
# ==========================================================================================================
desired_pos = ysw.DEMO_BASIC_POS_260826  
ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time, control_hold_time=0.5)

ysw.mobile_linear_distance(
    robot,
    distance_m=moved_distance_to_table,
    max_speed=0.25,
    accel=0.8,
    tolerance_m=0.01
)








