"""
    This is the Revised Demo File for 260826

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
    
    
    
    **********************************************************
        position control이나 mobile 관련 함수 실행 중 에러나서 멈추면,   
        - python3 poweroff.py                               
        - python3 initialize.py 
        실행 후 demoFile_260903.py 실행                        
    ***********************************************************
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
# Position Control Speed
position_control_minimum_time = 4      # [sec]



# Move to Table/shelf
vision_record_duration_tableDistance = 5.0   # [sec]
offset_table_distance = 0.4                  # [m]
vision_record_duration_shelfDistance = 5.0   # [sec]
offset_shelf_distance = 0.45                  # [m]



# Vision
vision_record_duration_checkBox = 5.0         # [sec]
vision_record_duration_checkObject = 5.0      # [sec]
box_minimum_area = 8000                       # [pixel^2]
box_maximum_distance = 1.2                    # [m]
object_minimum_area = 50                      # [pixel^2]
object_maximum_area = 1000                    # [pixel^2]
object_maximum_distance = 1.2                 # [m]



# Position Control 
offset_gripper_open_m = 0.015  # [m]
object_manual_offset_downToGrasp = -0.005      # [m] for z-direction offset when grasp
object_floating_z = 0.117 - object_manual_offset_downToGrasp     # [m]
object_manual_offset_downToGrasp_blackObj = -0.006      # [m] for z-direction offset when grasp
object_floating_z_blackObj  = 0.116 - object_manual_offset_downToGrasp_blackObj      # [m]
object_manual_offset_downToGrasp_redObj  = -0.010      # [m] for z-direction offset when grasp
object_floating_z_redObj = 0.112 - object_manual_offset_downToGrasp_redObj     # [m]
object_pregrasp_z_offset = +0.1   # [m]
object_grasp_x_offset = +0.014   # [m]

box_pregrasp_z_offset = +0.05   # [m]
box_grasp_z_offset = -0.065   # [m]
box_grasp_floating_offset = 0.08    # [m]
box_grasp_x_offset  = -0.04    # [m] outer side center point에서, 조금 더 앞쪽을 잡기 위한 offset 
box_grasp_y_offset  = 0.05    # [m] 그리퍼가 양쪽 모두 바깥 side보다 더 바깥쪽으로 배치되는 경향을 보정

box_move_readyPosition_right = np.array([0.55, -0.25, 1.2])      # desired position of "ee_finger_r1" from "base"  
box_move_readyPosition_left = np.array([0.55, 0.25, 1.2])        # desired position of "ee_finger_l1" from "base"  



# Gripper
grasp_current_obj = 0.15      # [A]
grasp_current_box = 0.25      # [A]
box_grasp_gripper_open_pos_mm = 80.0      # [mm]
gripper_open_ampere = -0.1    # [A]  
gripper_close_ampere = 0.1    # [A]



# ROI setting [Pixel Origin: (0,0) = 왼쪽 위 꼭짓점]
width = 640
height = 480

# table까지 거리를 잴 때 사용하는 ROI (Posture: PACK_POS)
pixel_roi_x_tableDistance = [140, width - 140] 
pixel_roi_y_tableDistance = [320, height]    

# table위 물체 detection에 사용하는 ROI (Posture: DEMO_BASIC_POS_260826)
pixel_roi_x_tableTop = None
# pixel_roi_y_tableTop = [150, height]
pixel_roi_y_tableTop = [150, height - 50]

# 박스를 grasp하고, 180도 돌아서 shelf까지 갈 때 사용하는 ROI 
pixel_roi_x_shelfDistance = [130, width - 130]      
pixel_roi_y_shelfDistance = [200, height]    



# ==========================================================================================================
# Initialization
# ==========================================================================================================
ysw.print_demoStep(step=1, text="Initialize")

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
time.sleep(1)

# ----------------------------------------------------------
# Depth Camera Initialization
# D435i:  Head
# D405_R: Right Wrist
# D405_L: Left Wrist
# ----------------------------------------------------------
d435i = ysw.RealSenseDepthCamera(rgb_on=True, depth_on=True, pixel=(640,480), fps=30, serial_number=ysw.HEAD_D435I_SERIAL)
d435i.initialize()
time.sleep(1)
d405_R = ysw.RealSenseDepthCamera(rgb_on=True, depth_on=True, pixel=(640,480), fps=30, serial_number=ysw.RIGHT_D405_SERIAL)
d405_R.initialize()
time.sleep(1)
d405_L = ysw.RealSenseDepthCamera(rgb_on=True, depth_on=True, pixel=(640,480), fps=30, serial_number=ysw.LEFT_D405_SERIAL)
d405_L.initialize()
time.sleep(1)



# ==========================================================================================================
# [Posture] Initial pose: PACK_POS
# ==========================================================================================================
ysw.print_demoStep(step=2, text="(Posture) PACK_POS")

desired_pos = ysw.PACK_POS  
ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time, control_hold_time=0.5)



# ==========================================================================================================
# [Vision] Check the Distance to Table

# Get the Distance of Center Pixel 
# Table distance is approximated as the depth of center pixel
# ==========================================================================================================
ysw.print_demoStep(step=3, text="(Vision) Check the distance to Table")

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
        if pixel_roi_x_tableDistance is None:
            x_min = 0
            x_max = w
        else:
            x_min, x_max = pixel_roi_x_tableDistance
            x_min = int(np.clip(x_min, 0, w - 1))
            x_max = int(np.clip(x_max, 0, w))

        if pixel_roi_y_tableDistance is None:
            y_min = 0
            y_max = h
        else:
            y_min, y_max = pixel_roi_y_tableDistance
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
# ==========================================================================================================
ysw.print_demoStep(step=4, text="(Posture) DEMO_BASIC_POS_260826")

desired_pos = ysw.DEMO_BASIC_POS_260826  
ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time, control_hold_time=0.5)



# ==========================================================================================================
# [Move] Move forward to Table
# ==========================================================================================================
ysw.print_demoStep(step=5, text="(Mobile) Move Forward to Table")

# expire stream 에러 가끔 나오는데 원인 불명
ysw.mobile_linear_distance(
    robot,
    distance_m=moved_distance_to_table,
    max_speed=0.25,
    accel=0.8,
    tolerance_m=0.01
)



# ==========================================================================================================
# Black Box check
# ==========================================================================================================
ysw.print_demoStep(step=6, text="(Vision) Black Box Detection")

result_blackBox = check_colored_box_and_set_grasp_arm(
    depthCamObj=d435i,
    boxColor="black",
    hsv_mask=ysw.HSV_MASK_BLACK_BOX,
    record_duration=vision_record_duration_checkBox,
    box_minimum_area=box_minimum_area,
    box_maximum_distance=box_maximum_distance,
    pixel_roi_x=pixel_roi_x_tableTop,
    pixel_roi_y=pixel_roi_y_tableTop,
    verbose=True,
)

blackBox_samples = result_blackBox["box_samples"]
blackBox_sample_avg = result_blackBox["box_sample_avg"]
grasp_arm_blackObj = result_blackBox["grasp_arm"]



# ==========================================================================================================
# Black Objects check (indexing from the center located object)
# ==========================================================================================================
ysw.print_demoStep(step=7, text="(Vision) Black Objects Detection")

result_blackObj = check_colored_objects_sorted_by_center(
    depthCamObj=d435i,
    objectColor="black",
    hsv_mask=ysw.HSV_MASK_BLACK_BOX,
    record_duration=vision_record_duration_checkObject,
    object_minimum_area=object_minimum_area,
    object_maximum_area=object_maximum_area,
    maximum_depth=object_maximum_distance,
    pixel_roi_x=pixel_roi_x_tableTop,
    pixel_roi_y=pixel_roi_y_tableTop,
    max_match_distance_pixel=40.0,
    min_rank_sample_ratio=0.4,
    min_rank_num_samples=50,
    verbose=True,
)

blackObj_sample_avg_list = result_blackObj["object_sample_avg_list"]



# ==========================================================================================================
# IK for Black Box and Objects
# ==========================================================================================================
ysw.print_demoStep(step=8, text="(IK) for Black Box and Objects")

black_target_result_list = []

for obj_idx, blackObj_sample_avg in enumerate(blackObj_sample_avg_list):
    if blackObj_sample_avg is None:
        print(f"[Black Object {obj_idx}] skipped: sample_avg is None")
        black_target_result_list.append(None)
        continue
    
    print(f"\n# ==================================================")
    print(f"# Compute IK for Black Object {obj_idx}")
    print(f"# ==================================================")
    
    black_target_result = compute_IK_for_colored_object_and_box(
        robot=robot,
        objectColor=f"black_{obj_idx}",
        object_sample_avg=blackObj_sample_avg,
        boxColor="black",
        box_sample_avg=blackBox_sample_avg,
        grasp_arm=grasp_arm_blackObj,
        object_manual_offset_downToGrasp=object_manual_offset_downToGrasp_blackObj,
        object_pregrasp_z_offset=object_pregrasp_z_offset,
        object_grasp_x_offset=object_grasp_x_offset, 
        box_grasp_z_offset=box_grasp_z_offset,
        box_grasp_x_offset=box_grasp_x_offset,
        box_grasp_y_offset=box_grasp_y_offset - 0.01,       # manual
        use_median=False,
        gripper_z_offset=ysw.GRIPPER_Z_OFFSET_FROM_LINK_EE_FINGER,
        verbose=True,
    )
    black_target_result_list.append(black_target_result)



# ==========================================================================================================
# Pick and Place for Black Box/Objects
# ==========================================================================================================
ysw.print_demoStep(step=9, text="[Pick and Place] Black Objects to Black Box")

for obj_idx, black_target_result in enumerate(black_target_result_list):
    if black_target_result is None:
        print(f"[Black Object {obj_idx}] skipped: target_result is None")
        continue

    print(f"\n# ==================================================")
    print(f"# Pick and Place Black Object {obj_idx}")
    print(f"# ==================================================")

    rv = pick_object_and_place_to_box(
        robot=robot,
        gripper=gripper,
        target_result=black_target_result,
        offset_gripper_open_m=offset_gripper_open_m,
        grasp_current_obj=grasp_current_obj,
        object_floating_z_afterGrasp=object_floating_z_blackObj,
        position_control_minimum_time=position_control_minimum_time,
        control_hold_time=0.5,
        fix_torso=True,
        verbose=True,
    )

    if rv is False:
        print(f"[Black Object {obj_idx}] Pick-place failed.")
        break
    
    
    
# ==========================================================================================================
# [Posture] Demo Basic pose: DEMO_BASIC_POS_260826
# ==========================================================================================================
ysw.print_demoStep(step=10, text="(Posture) DEMO_BASIC_POS_260826")

desired_pos = ysw.DEMO_BASIC_POS_260826  
ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time * 0.3, control_hold_time=0.5)



# ==========================================================================================================
# Red Box check
# ==========================================================================================================
ysw.print_demoStep(step=11, text="(Vision) Red Box Detection")

result_redBox = check_colored_box_and_set_grasp_arm(
    depthCamObj=d435i,
    boxColor="red",
    hsv_mask=ysw.HSV_MASK_RED_BOX,
    record_duration=vision_record_duration_checkBox,
    box_minimum_area=box_minimum_area,
    box_maximum_distance=box_maximum_distance,
    pixel_roi_x=pixel_roi_x_tableTop,
    pixel_roi_y=pixel_roi_y_tableTop,
    verbose=True,
)

redBox_samples = result_redBox["box_samples"]
redBox_sample_avg = result_redBox["box_sample_avg"]
grasp_arm_redObj = result_redBox["grasp_arm"]



# ==========================================================================================================
# Red Objects check (indexing from the center located object)
# ==========================================================================================================
ysw.print_demoStep(step=12, text="(Vision) Red Objects Detection")

result_redObj = check_colored_objects_sorted_by_center(
    depthCamObj=d435i,
    objectColor="red",
    hsv_mask=ysw.HSV_MASK_RED,
    record_duration=vision_record_duration_checkObject,
    object_minimum_area=object_minimum_area,
    object_maximum_area=object_maximum_area,
    maximum_depth=object_maximum_distance,
    pixel_roi_x=pixel_roi_x_tableTop,
    pixel_roi_y=pixel_roi_y_tableTop,
    max_match_distance_pixel=40.0,
    min_rank_sample_ratio=0.4,
    min_rank_num_samples=50,
    verbose=True,
)

redObj_sample_avg_list = result_redObj["object_sample_avg_list"]



# ==========================================================================================================
# IK for Red Box and Objects
# ==========================================================================================================
ysw.print_demoStep(step=13, text="(IK) for Red Box and Objects")

red_target_result_list = []

for obj_idx, redObj_sample_avg in enumerate(redObj_sample_avg_list):
    if redObj_sample_avg is None:
        print(f"[Red Object {obj_idx}] skipped: sample_avg is None")
        red_target_result_list.append(None)
        continue
    
    print(f"\n# ==================================================")
    print(f"# Compute IK for Red Object {obj_idx}")
    print(f"# ==================================================")
    
    red_target_result = compute_IK_for_colored_object_and_box(
        robot=robot,
        objectColor=f"red_{obj_idx}",
        object_sample_avg=redObj_sample_avg,
        boxColor="red",
        box_sample_avg=redBox_sample_avg,
        grasp_arm=grasp_arm_redObj,
        object_manual_offset_downToGrasp=object_manual_offset_downToGrasp_redObj,
        object_pregrasp_z_offset=object_pregrasp_z_offset,
        object_grasp_x_offset=object_grasp_x_offset - 0.001, # Manual 
        box_grasp_z_offset=box_grasp_z_offset - 0.008,       # Manual
        box_grasp_x_offset=box_grasp_x_offset - 0.005,       # Manual
        box_grasp_y_offset=box_grasp_y_offset,
        use_median=False,
        gripper_z_offset=ysw.GRIPPER_Z_OFFSET_FROM_LINK_EE_FINGER,
        verbose=True,
    )
    red_target_result_list.append(red_target_result)



# ==========================================================================================================
# Pick and Place for Red Box/Objects
# ==========================================================================================================
ysw.print_demoStep(step=14, text="[Pick and Place] Red Objects to Red Box")

for obj_idx, red_target_result in enumerate(red_target_result_list):
    if red_target_result is None:
        print(f"[Red Object {obj_idx}] skipped: target_result is None")
        continue

    print(f"\n# ==================================================")
    print(f"# Pick and Place Red Object {obj_idx}")
    print(f"# ==================================================")

    rv = pick_object_and_place_to_box(
        robot=robot,
        gripper=gripper,
        target_result=red_target_result,
        offset_gripper_open_m=offset_gripper_open_m,
        grasp_current_obj=grasp_current_obj,
        object_floating_z_afterGrasp=object_floating_z_redObj,
        position_control_minimum_time=position_control_minimum_time,
        control_hold_time=0.5,
        fix_torso=True,
        verbose=True,
    )

    if rv is False:
        print(f"[Red Object {obj_idx}] Pick-place failed.")
        break



# ==========================================================================================================
# [Posture] Demo Basic pose: DEMO_BASIC_POS_260826
# ==========================================================================================================
ysw.print_demoStep(step=15, text="(Posture) DEMO_BASIC_POS_260826")

desired_pos = ysw.DEMO_BASIC_POS_260826  
ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time * 0.3, control_hold_time=0.5)



# ==========================================================================================================
# Grasp Black and Red Box's outer side and slightly lift
# ==========================================================================================================
ysw.print_demoStep(step=16, text="(Pick Boxes) grasp outer side and slightly lift [Black & Red] Boxes")

# Black Box Grasp and lift
rv = grasp_box_outerside_and_lift(
    robot=robot,
    gripper=gripper,
    target_result=black_target_result_list[0],
    box_grasp_current=grasp_current_box,
    box_grasp_gripper_open_pos_mm=box_grasp_gripper_open_pos_mm,
    box_floating_z=box_grasp_floating_offset,
    position_control_minimum_time=position_control_minimum_time,
    control_hold_time=0.5,
    fix_torso=True,
    verbose=True,
)

# Red Box Grasp an lift
rv = grasp_box_outerside_and_lift(
    robot=robot,
    gripper=gripper,
    target_result=red_target_result_list[0],
    box_grasp_current=grasp_current_box,
    box_grasp_gripper_open_pos_mm=box_grasp_gripper_open_pos_mm,
    box_floating_z=box_grasp_floating_offset,
    position_control_minimum_time=position_control_minimum_time,
    control_hold_time=0.5,
    fix_torso=True,
    verbose=True,
)



# ==========================================================================================================
# [Posture] DEMO_MOVE_POS_260826
# ==========================================================================================================
ysw.print_demoStep(step=17, text="(Posture) DEMO_MOVE_POS_260826")

desired_pos = ysw.DEMO_MOVE_POS_260826  
ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time * 0.6, control_hold_time=0.5)



# ==========================================================================================================
# Change the posture to Box-move-to-shelf Ready Posture
# ==========================================================================================================
ysw.print_demoStep(step=18, text="(Posture) Box-move-to-shelf Ready Posture")

# For Black Box
rv = move_ee_to_box_lift_ready_pose(
    robot=robot,
    target_result=black_target_result_list[0],
    pos_right=box_move_readyPosition_right,      # manual setting
    pos_left=box_move_readyPosition_left,        # manual setting
    target_T_key="T_SC_target_for_box_side_after_open_gripper",
    minimum_time=position_control_minimum_time * 0.6,
    control_hold_time=0.5,
    get_return=True,
)

# For Red Box
rv = move_ee_to_box_lift_ready_pose(
    robot=robot,
    target_result=red_target_result_list[0],
    pos_right=box_move_readyPosition_right,      # manual setting
    pos_left=box_move_readyPosition_left,        # manual setting
    target_T_key="T_SC_target_for_box_side_after_open_gripper",
    minimum_time=position_control_minimum_time * 0.6,
    control_hold_time=0.5,
    get_return=True,
)



# ==========================================================================================================
# [Mobile] Slightly move backward to turn
# ==========================================================================================================
ysw.print_demoStep(step=19, text="(Mobile) Move Backward slightly")

moved_distance_to_back = -0.7       # manually changed

ysw.mobile_linear_distance(
    robot,
    distance_m=moved_distance_to_back,
    max_speed=0.12,
    accel=0.1,
    tolerance_m=0.01
)
time.sleep(1)



# ==========================================================================================================
# [Mobile] Turn Right 180 degree (to go shelf)
# ==========================================================================================================
ysw.print_demoStep(step=20, text="(Mobile) Turn Right 180 degree")

desired_yaw_deg = -176                   # Unit: [deg], (+): CCW, (-): CW

ysw.mobile_turn_inplace(
    robot,
    angle_deg=desired_yaw_deg, 
    max_yaw_vel=0.6,
    yaw_accel=1.5,
    tolerance_deg=1.0
)



# ==========================================================================================================
# [Vision] Check the distance to shelf
# Get the Distance of Table using nearest depth average in ROI
# ==========================================================================================================
ysw.print_demoStep(step=21, text="(Vision) Check the distance to shelf")

record_duration = vision_record_duration_shelfDistance      # [sec]

distance_list = []
distance_median_list = []
distance_min_list = []

rv = d435i.startPipeline()
if rv is False:
    raise RuntimeError("[D435i] Failed to start pipeline.")

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

        if color_image is None or depth_image is None:
            continue

        nearest_depth_result = get_nearest_depth_avg_in_roi(
            depth_frame=depth_frame,
            pixel_roi_x=pixel_roi_x_shelfDistance,
            pixel_roi_y=pixel_roi_y_shelfDistance,
            nearest_ratio=0.05,      # 가장 가까운 5%
            min_num_points=30,       # 최소 30개 pixel 평균
            max_valid_depth=3.0,     # 필요하면 2.0 [m] 이런 식으로 제한 가능
        )

        if nearest_depth_result is None:
            print("[WARNING] No valid depth in ROI.")
            continue

        nearest_depth_avg = nearest_depth_result["nearest_depth_avg"]
        nearest_depth_median = nearest_depth_result["nearest_depth_median"]
        nearest_depth_min = nearest_depth_result["nearest_depth_min"]

        distance_list.append(nearest_depth_avg)
        distance_median_list.append(nearest_depth_median)
        distance_min_list.append(nearest_depth_min)

        print(
            f"Table Distance nearest_avg: {nearest_depth_avg:.3f} [m], "
            f"median: {nearest_depth_median:.3f} [m], "
            f"min: {nearest_depth_min:.3f} [m], "
            f"used: {nearest_depth_result['num_used_depth']}, "
            f"valid: {nearest_depth_result['num_valid_depth']}"
        )

finally:
    d435i.stopPipeline()
    d435i.destroyAllWindows()

# ------------------
# Average Distance
#-------------------
if len(distance_list) == 0:
    raise RuntimeError("[ERROR] No valid table distance was measured.")

distance_avg = float(np.mean(distance_list))
distance_median = float(np.median(distance_list))
distance_std = float(np.std(distance_list))

print("\n========== Shelf Distance Result ==========")
print(f"num_samples           : {len(distance_list)}")
print(f"Shelf Average Distance: {distance_avg:.4f} [m]")
print(f"Shelf Median Distance : {distance_median:.4f} [m]")
print(f"Shelf Distance Std    : {distance_std:.4f} [m]")

moved_distance_to_shelf = distance_avg - offset_shelf_distance

print(f"\noffset_shelf_distance : {offset_shelf_distance:.4f} [m]")
print(f"moved_distance_to_shelf: {moved_distance_to_shelf:.4f} [m]")



# ==========================================================================================================
# [Mobile] Move Forward to Shelf
# ==========================================================================================================
ysw.print_demoStep(step=22, text="(Mobile) Move Forward to Shelf")

ysw.mobile_linear_distance(
    robot,
    distance_m=moved_distance_to_shelf,
    max_speed=0.25,
    accel=0.8,
    tolerance_m=0.01
)



# ==========================================================================================================
# [Gripper] Open grippers (Lay Boxes) and Close grippers
# ==========================================================================================================
ysw.print_demoStep(step=23, text="(Gripper) Lay Boxes")

gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_L, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
gripper.write_current_ampere(dxl_id=ysw.GRIPPER_ID_L, current_ampere=gripper_open_ampere)
time.sleep(1)

gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
gripper.write_current_ampere(dxl_id=ysw.GRIPPER_ID_R, current_ampere=gripper_open_ampere)
time.sleep(1)

gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_L, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
gripper.write_current_ampere(dxl_id=ysw.GRIPPER_ID_L, current_ampere=gripper_close_ampere)
time.sleep(1)

gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
gripper.write_current_ampere(dxl_id=ysw.GRIPPER_ID_R, current_ampere=gripper_close_ampere)
time.sleep(1)




# ==========================================================================================================
# [Mobile] Go initial states
# ==========================================================================================================
ysw.print_demoStep(step=24, text="(Mobile) Move Backward slightly and Turn Left 180 degree")

moved_distance_to_back = -0.7
ysw.mobile_linear_distance(
    robot,
    distance_m=moved_distance_to_back,
    max_speed=0.25,
    accel=0.8,
    tolerance_m=0.01
)
time.sleep(1)

desired_yaw_deg = +176                   # Unit: [deg], (+): CCW, (-): CW
ysw.mobile_turn_inplace(
    robot,
    angle_deg=desired_yaw_deg, 
    max_yaw_vel=0.6,
    yaw_accel=1.5,
    tolerance_deg=1.0
)
time.sleep(1)



# ==========================================================================================================
# [Posture] DEMO_MOVE_POS_260826
# ==========================================================================================================
ysw.print_demoStep(step=25, text="(Posture) DEMO_MOVE_POS_260826")

desired_pos = ysw.PACK_POS  
ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time, control_hold_time=0.5)





