"""
This is the Demo File for 260723

[Scenario]
1. In PML Lab, the robot turns left 90 degrees and takes the leftmost black block
    (1) If the leftmost black block is in left side from the center of robot:
        Use its left arm.
    (2) If the leftmost black block is in right side from the center of robot:
        Use its right arm.
2. The robot turns right 90 degrees and check the yellow collecting box's position
    (1) If the box is in the left side, and the block is in the left arm:
        Lay the block by using current(left) arm.
    (2) If the box is in the right side, and the block is in the left arm:
        Transfers the object from the left arm to the right arm. Lay the block by using tranferred(right) arm.
    (3) If the box is in the left side, and the block is in the right arm:
        Transfers the object from the right arm to the left arm. Lay the block by using tranferred(left) arm.
    (4) If the box is in the right side, and the block is in the right arm:
        Lay the block by using current(right) arm.

[case_list]
(grasp side, box side)
(grasp_arm, lay_arm)

"left-to-left": grasp using left arm and lay using left arm
"left_to_right"
"right_to_left" 
"right_to_right"
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
# Path.cwd()           : folder "260723", 
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
position_control_minimum_time = 5.5      # [sec]
minimum_time_scaler = 0.5                # For short movement
grasp_current = 0.2     # [A]
object_floating_x_in_linktorso5 = -0.05   # [m]
object_floating_z_in_linktorso5 = 0.05    # [m]
vision_record_duration = 2.0    # [sec]

trial_num = 4       # How many times to do -> It can be regarded as the object numbers to "pick & place"          


# ==========================================================================================================
# Initialize
# ==========================================================================================================
#########
# Robot #
#########
robotConfig = ysw.setRobotConfig(ysw.MODE_DIRECT_UPC, ysw.MODEL_A)          # For Real Robot + UPC

robot = ysw.initialize(robotConfig.mode, robotConfig.address, robotConfig.model, option=ysw.CONNECT_AND_POWER)
if (robot != None):
    model = robot.model()
    
time.sleep(1)

###########
# Gripper #
###########
# Gripper Setting (Need to hard check of gripper ping test)
gripper = ysw.Gripper()
rv1 = False
rv2 = False

while not (rv1 and rv2):
    print("Trying to connect gripper...")
    rv1 = gripper.connect(device_name=ysw.GRIPPER_DEVICE_NAME, protocol_version=ysw.DXL_PROTOCOL_VERSION, baudrate=ysw.DXL_BAUDRATE)
    
    if rv1:
        rv2 = gripper.test_ping()
    else:
        rv2 = False
        
    print(f"[Gripper] (connect) = {rv1}, (ping) = {rv2}")
    
    if rv1 and rv2:
        break
    
time.sleep(1)


# ==========================================================================================================
# Initial Posture setting
# ==========================================================================================================
desired_pos = ysw.DEMO_BASIC_POS_260723   
ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time, control_hold_time=0.5)




# ==========================================================================================================
# Iterative Tasks for "trial_num"
# ==========================================================================================================
trial_idx = 0

while (trial_idx < trial_num):
    # ======================================================================================================
    # Turn to Left Shelf
    # ======================================================================================================
    desired_pos = ysw.DEMO_TURN_POS_260723    
    ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time, control_hold_time=0.5)


    # ======================================================================================================
    # Detection of the Leftmost Black Object during "vision_record_duration" seconds
    # ======================================================================================================
    #####################################
    # ROI setting                       #
    # Pixel Origin: (0,0) = 왼쪽 위 꼭짓점 #
    #####################################
    width = 640
    height = 480
    pixel_roi_x = None            # None이면 x 전체 사용
    pixel_roi_y = [170, height]   # y는 170~480만 사용


    #########
    # Start #
    #########
    d435i = ysw.D435i(rgb_on=True, depth_on=True, pixel=(640, 480), fps=30)
    d435i.initialize()

    record_duration = vision_record_duration  # [sec]

    leftmost_object_samples = []        # 제일 왼쪽 물체

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

            detected_objects, color_image, mask = d435i.detectColorObjectContour3D(
                color_image=color_image,
                depth_frame=depth_frame,
                mask_list=ysw.HSV_MASK_BLACK,
                minimum_area=1000,       
                maximum_depth=0.8,                   # ignore distant object larger than x[m] 
                pixel_roi_x=pixel_roi_x,
                pixel_roi_y=pixel_roi_y,
            )

            if len(detected_objects) == 0:
                continue

            # 카메라 기준 X 좌표가 작을수록 이미지 왼쪽
            # RealSense/D435i 기준: X+ = image right
            obj_list_from_left = sorted(
                detected_objects,
                key=lambda obj: obj["center_3d"][0],
                reverse=False
            )

            # 제일 왼쪽 물체 하나만 선택
            obj = obj_list_from_left[0]

            # 제일 왼쪽 물체 sample만 저장
            leftmost_object_samples.append({
                "area_2d":      float(obj["area_2d"]),
                "pixel_center": np.array(obj["pixel_center"], dtype=np.float64),
                "center_3d":    np.array(obj["center_3d"], dtype=np.float64),
                "bbox_3d_size": np.array(obj["bbox_3d_size"], dtype=np.float64),
                "bbox_3d_min":  np.array(obj["bbox_3d_min"], dtype=np.float64),
                "bbox_3d_max":  np.array(obj["bbox_3d_max"], dtype=np.float64),
            })

    finally:
        d435i.stopPipeline()
        d435i.destroyAllWindows()

    ###########################
    # Average leftmost object #
    ###########################
    leftmost_obj_avg_sample = None

    if len(leftmost_object_samples) > 0:
        leftmost_obj_avg_sample = ysw.average_detectColorObjectContour3D(
            leftmost_object_samples
        )

    print("\n========== Detection Result ==========")
    print("Only the LEFTMOST object is used\n")

    if leftmost_obj_avg_sample is None:
        print("No object detected.")

    else:
        print("\n[Leftmost Object]")
        print(f"num_samples             : {leftmost_obj_avg_sample['num_samples']}")
        print(f"area_2d_avg             : {leftmost_obj_avg_sample['area_2d_avg']:.1f}")
        print(f"area_2d_median          : {leftmost_obj_avg_sample['area_2d_median']:.1f}")
        print(f"center_3d_avg           : {np.round(leftmost_obj_avg_sample['center_3d_avg'], 4)} m")
        print(f"center_3d_median        : {np.round(leftmost_obj_avg_sample['center_3d_median'], 4)} m")
        print(f"bbox_3d_size_avg        : {np.round(leftmost_obj_avg_sample['bbox_3d_size_avg'], 4)} m")
        print(f"bbox_3d_size_median     : {np.round(leftmost_obj_avg_sample['bbox_3d_size_median'], 4)} m")
        

    # ======================================================================================================
    # Calculate IK for the leftmost black object respect to "link_torso_5"
    # ======================================================================================================
    # {A}: vision frame
    # {B}: "link_head_2" frame
    # {S}: "link_torso_5" frame 
    # {M}: Goal frame (object) 
    # {M1}: Goal frame (object) before open the gripper 
    # {M2}: Goal frame (object) after open the gripper 
    # {N}: Goal frame (collecting box) 
    # {C}: ee_finger_l1/r1 frame
    # {D}: final fingertip frame -> same orientation of {C}

    manual_offset = 0.05  # [m] for forward x-direction relative to "link_torso_5"

    leftmost_obj_pos    = leftmost_obj_avg_sample["center_3d_avg"]
    leftmost_obj_3Dsize = leftmost_obj_avg_sample["bbox_3d_size_avg"]

    P_BM = ysw.convertPos_vision_to_linkhead2(leftmost_obj_pos)

    fk_result = ysw.get_FK(robot, "link_torso_5", "link_head_2")
    T_SB = fk_result["T"]
    R_SB = fk_result["rotation"]
    P_SB = fk_result["position"]

    P_BM_extended = np.append(P_BM, 1.0)
    P_SM = T_SB @ P_BM_extended
    P_SM = P_SM[:3]

    grasp_arm = None
    if (P_BM[1] <= 0):     
        grasp_arm = "right"
    else:     
        grasp_arm = "left"

    if (grasp_arm == "right"):
        R1 = ysw.rotMat_z(90.0*ysw.DEG2RAD)      # "link_torso_5" frame 기준으로 z축 +90도 회전
        R2 = ysw.rotMat_x(-90.0*ysw.DEG2RAD)     # After R1, converted "link_torso_5" frame 기준으로 x축 -90도 회전
        R_SC_desired = R1 @ R2                  
    elif (grasp_arm == "left"):
        R1 = ysw.rotMat_x(90.0*ysw.DEG2RAD)      # "link_torso_5" frame 기준으로 x축 +90도 회전
        R2 = ysw.rotMat_y(-90.0*ysw.DEG2RAD)     # After R1, converted "link_torso_5" frame 기준으로 y축 -90도 회전
        R_SC_desired = R1 @ R2                  

    P_SM1 = P_SM + np.array([-abs(ysw.GRIPPER_OFFSET_BEFORE_GRASP), 0.0, 0.0])      # x-offset before open the gripper
    R_SM1 = R_SC_desired
    T_SM1 = ysw.make_HT(R_SM1, P_SM1)    

    P_SM2 = P_SM + np.array([manual_offset, 0.0, 0.0])      # Theoretically manual_offset should be zero
    R_SM2 = R_SC_desired
    T_SM2 = ysw.make_HT(R_SM2, P_SM2)    

    # "ee_finger_l1/r1" to final end-effector -> should be physically measured
    pos_from_eeFinger_to_fingertip = np.array([0, 0, ysw.GRIPPER_Z_OFFSET_FROM_LINK_EE_FINGER]) # P_CD
    P_CD = pos_from_eeFinger_to_fingertip
    T_CD = ysw.make_HT(np.eye(3), P_CD)    

    T_SC_target_before_open_gripper = T_SM1 @ np.linalg.inv(T_CD)
    T_SC_target_after_open_gripper  = T_SM2 @ np.linalg.inv(T_CD)
    print(f"Before open gripper: \n{T_SC_target_before_open_gripper}\n")
    print(f"After open gripper: \n{T_SC_target_after_open_gripper}\n")
    print(f"Grasping Arm: {grasp_arm}")

        
        
    # ======================================================================================================
    # Arm is moving to the leftmost black object on shelf (Before open the gripper)
    # ======================================================================================================
    pos_before_open_gripper = T_SC_target_before_open_gripper[:3,3]
    ori_before_open_gripper = ysw.rotMat_to_rpy(T_SC_target_before_open_gripper[:3,:3])       

    if (grasp_arm == "right"):      # Object is in right side of robot
        cartesian_position_targets_sample = [    
                                                {
                                                    "reference_link": "link_torso_5",
                                                    "target_link": "ee_finger_r1",
                                                    "pos": pos_before_open_gripper,                  
                                                    "rpy": ori_before_open_gripper,                    
                                                },                 
                                            ]
        
    elif (grasp_arm == "left"):       # Object is in left side of robot
        cartesian_position_targets_sample = [    
                                                {
                                                    "reference_link": "link_torso_5",
                                                    "target_link": "ee_finger_l1",
                                                    "pos": pos_before_open_gripper,                  
                                                    "rpy": ori_before_open_gripper,                    
                                                },                 
                                            ]


    # [rv = "function"] form is Necessary when "get_return=False"
    rv = ysw.positionCtrl_cartesian_wholeBody(
        robot, 
        position_targets=cartesian_position_targets_sample, 
        joint_targets=None,
        minimum_time=position_control_minimum_time * 0.7,
        control_hold_time=0.5,
        get_return=True
    )


    # ======================================================================================================
    # Open the Gripper Before grasp
    # ======================================================================================================
    object_meanDim = np.mean(leftmost_obj_3Dsize)       # mean value of 3D size of object to grasp
    desiredGripperPos = object_meanDim 

    # desiredGripperPos_mm = desiredGripperPos * 1000.0 + ysw.GRIPPER_SAFETY_OPEN_OFFSET
    desiredGripperPos_mm = desiredGripperPos * 1000.0
    if desiredGripperPos_mm >= ysw.GRIPPER_POS_RANGE_LINEAR:
        desiredGripperPos_mm = ysw.GRIPPER_POS_RANGE_LINEAR - 10.0

    if (grasp_arm == "right"):      # Object is in right side of robot
        gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE)
        gripper.current_based_position_control_linear(ysw.GRIPPER_ID_R, current_ampere_mag=0.02, linear_pos=desiredGripperPos_mm)
        time.sleep(1)
        gripper.disable(ysw.GRIPPER_ID_R)
    elif (grasp_arm == "left"):       # Object is in left side of robot
        gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_L, operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE)
        gripper.current_based_position_control_linear(ysw.GRIPPER_ID_L, current_ampere_mag=0.02, linear_pos=desiredGripperPos_mm)
        time.sleep(1)
        gripper.disable(ysw.GRIPPER_ID_L)
        
    
    # ======================================================================================================
    # Arm is moving to the leftmost black object on shelf (After open the gripper) -> Remained distance
    # ======================================================================================================
    pos_after_open_gripper = T_SC_target_after_open_gripper[:3,3]
    ori_after_open_gripper = ysw.rotMat_to_rpy(T_SC_target_after_open_gripper[:3,:3])       

    if (grasp_arm == "right"):      # Object is in right side of robot
        cartesian_position_targets_sample = [    
                                                {
                                                    "reference_link": "link_torso_5",
                                                    "target_link": "ee_finger_r1",
                                                    "pos": pos_after_open_gripper,                  
                                                    "rpy": ori_after_open_gripper,                    
                                                },                 
                                            ]
        
    elif (grasp_arm == "left"):       # Object is in left side of robot
        cartesian_position_targets_sample = [    
                                                {
                                                    "reference_link": "link_torso_5",
                                                    "target_link": "ee_finger_l1",
                                                    "pos": pos_after_open_gripper,                  
                                                    "rpy": ori_after_open_gripper,                    
                                                },                 
                                            ]


    # [rv = "function"] form is Necessary when "get_return=False"
    rv = ysw.positionCtrl_cartesian_wholeBody(
        robot, 
        position_targets=cartesian_position_targets_sample, 
        joint_targets=None,
        minimum_time=position_control_minimum_time * 0.3,
        control_hold_time=0.5,
        get_return=True
    )
    
    
    # ======================================================================================================
    # Close the Gripper to grasp
    # ======================================================================================================
    if (grasp_arm == "right"):      # Object is in right side of robot
        gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
        gripper.write_current_ampere(ysw.GRIPPER_ID_R, grasp_current)
    elif (grasp_arm == "left"):       # Object is in left side of robot
        gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_L, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
        gripper.write_current_ampere(ysw.GRIPPER_ID_L, grasp_current)
    time.sleep(1)
    
    
    # ======================================================================================================
    # Slightly float the object to avoid Collision
    # ======================================================================================================
    if (grasp_arm == "right"):      # Object is in right side of robot
        cartesian_position_targets_sample = [    
                                                {
                                                    "reference_link": "link_torso_5",
                                                    "target_link": "ee_finger_r1",
                                                    "pos": pos_after_open_gripper + np.array([object_floating_x_in_linktorso5, 0.0, object_floating_z_in_linktorso5]),                  
                                                    "rpy": ori_after_open_gripper,                    
                                                },                 
                                            ]
        
    elif (grasp_arm == "left"):       # Object is in left side of robot
        cartesian_position_targets_sample = [    
                                                {
                                                    "reference_link": "link_torso_5",
                                                    "target_link": "ee_finger_l1",
                                                    "pos": pos_after_open_gripper + np.array([object_floating_x_in_linktorso5, 0.0, object_floating_z_in_linktorso5]),                  
                                                    "rpy": ori_after_open_gripper,                    
                                                },                 
                                            ]


    # [rv = "function"] form is Necessary when "get_return=False"
    rv = ysw.positionCtrl_cartesian_wholeBody(
        robot, 
        position_targets=cartesian_position_targets_sample, 
        joint_targets=None,
        minimum_time=position_control_minimum_time * 0.5,
        control_hold_time=0.5,
        get_return=True
    )
    
    
    # ======================================================================================================
    # Pull the grasp arm close to Torso before Turn Right (to avoid collision)
    # ======================================================================================================
    desired_pos = ysw.DEMO_AFTER_GRASP_POS_260723 
    ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time*0.5, control_hold_time=0.5)
    
    # ======================================================================================================
    # Turn Right to face Table (Initial Posture)
    # ======================================================================================================
    desired_pos = ysw.DEMO_BASIC_POS_260723 
    ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time, control_hold_time=0.5)
    
    # ======================================================================================================
    # Move both arms apart slightly to ensure the Vision range 
    # ======================================================================================================
    desired_pos = ysw.DEMO_VISION_BASIC_POS_260723 
    ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time*0.3, control_hold_time=0.5)
    
    
    # ======================================================================================================
    # Detection of Yellow Collecting Box during "vision_record_duration" seconds
    # ======================================================================================================
    d435i = ysw.D435i(rgb_on=True, depth_on=True, pixel=(640, 480), fps=30)
    d435i.initialize()

    record_duration = vision_record_duration  # [sec]

    box_samples = []        # box

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

            detected_objects, color_image, mask = d435i.detectColorObjectContour3D(
                color_image=color_image,
                depth_frame=depth_frame,
                mask_list=ysw.HSV_MASK_YELLOW,
                minimum_area=10000,       
                maximum_depth=1.5,                   # ignore distant object larger than x[m] 
            )

            if len(detected_objects) == 0:
                continue

            # The largest yellow object = box
            largest_obj = max(
                detected_objects,
                key=lambda obj: obj["area_2d"]
            )

            # [Largest area] sample
            box_samples.append({
                "area_2d":      float(largest_obj["area_2d"]),
                "pixel_center": np.array(largest_obj["pixel_center"], dtype=np.float64),
                "center_3d":    np.array(largest_obj["center_3d"], dtype=np.float64),
                "bbox_3d_size": np.array(largest_obj["bbox_3d_size"], dtype=np.float64),
                "bbox_3d_min":  np.array(largest_obj["bbox_3d_min"], dtype=np.float64),
                "bbox_3d_max":  np.array(largest_obj["bbox_3d_max"], dtype=np.float64),
            })

    finally:
        d435i.stopPipeline()
        d435i.destroyAllWindows()

    ######################
    # Average Yellow Box #
    ######################
    box_obj_avg_sample = None

    if len(box_samples) > 0:
        box_obj_avg_sample = ysw.average_detectColorObjectContour3D(
            box_samples
        )

    print("\n========== Detection Result ==========")
    print("Only the Largest Box is used\n")

    if box_obj_avg_sample is None:
        print("No object detected.")

    else:
        print("\n[Largest Box Object]")
        print(f"num_samples             : {box_obj_avg_sample['num_samples']}")
        print(f"area_2d_avg             : {box_obj_avg_sample['area_2d_avg']:.1f}")
        print(f"area_2d_median          : {box_obj_avg_sample['area_2d_median']:.1f}")
        print(f"center_3d_avg           : {np.round(box_obj_avg_sample['center_3d_avg'], 4)} m")
        print(f"center_3d_median        : {np.round(box_obj_avg_sample['center_3d_median'], 4)} m")
        print(f"bbox_3d_size_avg        : {np.round(box_obj_avg_sample['bbox_3d_size_avg'], 4)} m")
        print(f"bbox_3d_size_median     : {np.round(box_obj_avg_sample['bbox_3d_size_median'], 4)} m")
    
    
    # ======================================================================================================
    # Calculate IK for Yellow Box respect to "link_torso_5" frame
    # ======================================================================================================
    # {A}: vision frame
    # {B}: "link_head_2" frame
    # {S}: "link_torso_5" frame 
    # {N}: Goal frame (collecting box) 
    # {N1}: Goal frame (collecting box) 
    # {N2}: Goal frame (collecting box) 
    # {C}: ee_finger_l1/r1 frame
    # {D}: final fingertip frame -> same orientation of {C}

    box_pos    = box_obj_avg_sample["center_3d_avg"]
    box_3Dsize = box_obj_avg_sample["bbox_3d_size_avg"]

    P_BN = ysw.convertPos_vision_to_linkhead2(box_pos)

    fk_result = ysw.get_FK(robot, "link_torso_5", "link_head_2")
    T_SB = fk_result["T"]
    R_SB = fk_result["rotation"]
    P_SB = fk_result["position"]

    P_BN_extended = np.append(P_BN, 1.0)
    P_SN = T_SB @ P_BN_extended
    P_SN = P_SN[:3]

    lay_arm = None
    if (P_BN[1] <= 0):     
        lay_arm = "right"
    else:     
        lay_arm = "left"
            
    if (lay_arm == "left"):
        R1 = ysw.rotMat_x(90.0*ysw.DEG2RAD)      # "link_torso_5" frame 기준으로 x축 +90도 회전
        R2 = ysw.rotMat_y(-90.0*ysw.DEG2RAD)     # After R1, converted "link_torso_5" frame 기준으로 y축 -90도 회전
        R_SC_desired = R1 @ R2                   
    elif (lay_arm == "right"):
        R1 = ysw.rotMat_z(90.0*ysw.DEG2RAD)      # "link_torso_5" frame 기준으로 z축 +90도 회전
        R2 = ysw.rotMat_x(-90.0*ysw.DEG2RAD)     # After R1, converted "link_torso_5" frame 기준으로 x축 -90도 회전
        R_SC_desired = R1 @ R2               

    R_SN = R_SC_desired
    T_SN = ysw.make_HT(R_SN, P_SN)    

    # "ee_finger_l1/r1" to final end-effector -> should be physically measured
    pos_from_eeFinger_to_fingertip = np.array([0, 0, ysw.GRIPPER_Z_OFFSET_FROM_LINK_EE_FINGER]) # P_CD
    P_CD = pos_from_eeFinger_to_fingertip
    T_CD = ysw.make_HT(np.eye(3), P_CD)    

    T_SC_target_for_box = T_SN @ np.linalg.inv(T_CD)
    print(f"For Yellow Box: \n{T_SC_target_for_box}\n")
    print(f"Laying Arm: {lay_arm}")





    # ======================================================================================================
    # [MAIN] Case-by-Case Movements !!!
    # ======================================================================================================
    if (grasp_arm == "left" and lay_arm == "left"):
        # =======================================================================
        # 1. Move to Yellow Box by using Left arm
        # =======================================================================
        pos_box = T_SC_target_for_box[:3,3]
        ori_box = ysw.rotMat_to_rpy(T_SC_target_for_box[:3,:3])       

        cartesian_position_targets_sample = [    
                                                {
                                                    "reference_link": "link_torso_5",
                                                    "target_link": "ee_finger_l1",
                                                    "pos": pos_box + np.array([0.0, 0.0, (np.mean(box_3Dsize)/1.5)]),
                                                    "rpy": ori_box                    
                                                },                 
                                            ]
        # [rv = "function"] form is Necessary when "get_return=False"
        rv = ysw.positionCtrl_cartesian_wholeBody(
            robot, 
            position_targets=cartesian_position_targets_sample, 
            joint_targets=None,
            minimum_time=position_control_minimum_time,
            control_hold_time=0.5,
            get_return=True
        )   
        # =======================================================================
        # 2. Open Left gripper to Lay
        # ======================================================================= 
        gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_L, operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE)
        gripper.current_based_position_control_linear(ysw.GRIPPER_ID_L, current_ampere_mag=0.02, linear_pos=desiredGripperPos_mm)
        time.sleep(1)
        gripper.disable(ysw.GRIPPER_ID_L)
        
    elif (grasp_arm == "left" and lay_arm == "right"):
        # =======================================================================
        # 1. Move to pre-[L-to-R] posture
        # =======================================================================
        desired_pos = ysw.DEMO_LEFT_TO_RIGHT_POS_BEFORE_260723  
        ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time, control_hold_time=0.5)
        
        # =======================================================================
        # 2. Open Right Gripper before [L-to-R] gripper transfer
        # =======================================================================
        object_maxDim = max(leftmost_obj_3Dsize)       # max value of 3D size of object to grasp
        desiredGripperPos = object_maxDim 

        desiredGripperPos_mm = desiredGripperPos * 1000.0
        if desiredGripperPos_mm >= ysw.GRIPPER_POS_RANGE_LINEAR:
            desiredGripperPos_mm = ysw.GRIPPER_POS_RANGE_LINEAR - 8.0

        gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE)
        gripper.current_based_position_control_linear(ysw.GRIPPER_ID_R, current_ampere_mag=0.02, linear_pos=desiredGripperPos_mm)
        time.sleep(1)
        gripper.disable(ysw.GRIPPER_ID_R)
            
        # =======================================================================
        # 3. Move to post-[L-to-R] posture
        # =======================================================================
        desired_pos = ysw.DEMO_LEFT_TO_RIGHT_POS_AFTER_260723  
        ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time * 0.3, control_hold_time=0.5)
        
        # =======================================================================
        # 4. Close Right Gripper & Open Left Gripper
        # =======================================================================
        gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
        gripper.write_current_ampere(ysw.GRIPPER_ID_R, grasp_current)
        time.sleep(1)
        gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_L, operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE)
        gripper.current_based_position_control_linear(ysw.GRIPPER_ID_L, current_ampere_mag=0.02, linear_pos=desiredGripperPos_mm)
        time.sleep(1)
        gripper.disable(ysw.GRIPPER_ID_L)
        
        # =======================================================================
        # 5. Move to pre-[L-to-R] posture
        # =======================================================================
        desired_pos = ysw.DEMO_LEFT_TO_RIGHT_POS_BEFORE_260723  
        ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time * 0.3, control_hold_time=0.5)
            
        # =======================================================================
        # 6. Move to Yellow Box by using Right arm
        # =======================================================================
        pos_box = T_SC_target_for_box[:3,3]
        ori_box = ysw.rotMat_to_rpy(T_SC_target_for_box[:3,:3])       

        cartesian_position_targets_sample = [    
                                                {
                                                    "reference_link": "link_torso_5",
                                                    "target_link": "ee_finger_r1",
                                                    "pos": pos_box + np.array([0.0, 0.0, (np.mean(box_3Dsize)/1.5)]),
                                                    "rpy": ori_box                    
                                                },                 
                                            ]
        # [rv = "function"] form is Necessary when "get_return=False"
        rv = ysw.positionCtrl_cartesian_wholeBody(
            robot, 
            position_targets=cartesian_position_targets_sample, 
            joint_targets=None,
            minimum_time=position_control_minimum_time,
            control_hold_time=0.5,
            get_return=True
        )   
        
        # =======================================================================
        # 7. Open Right gripper to Lay
        # ======================================================================= 
        gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE)
        gripper.current_based_position_control_linear(ysw.GRIPPER_ID_R, current_ampere_mag=0.02, linear_pos=desiredGripperPos_mm)
        time.sleep(1)
        gripper.disable(ysw.GRIPPER_ID_R)
            
    elif (grasp_arm == "right" and lay_arm == "left"):
        # =======================================================================
        # 1. Need to pre-[R-to-L] posture
        # =======================================================================
        desired_pos = ysw.DEMO_RIGHT_TO_LEFT_POS_BEFORE_260723  
        ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time, control_hold_time=0.5)
        
        # =======================================================================
        # 2. Open Left Gripper before [R-to-L] gripper transfer
        # =======================================================================
        object_maxDim = max(leftmost_obj_3Dsize)       # max value of 3D size of object to grasp
        desiredGripperPos = object_maxDim 

        desiredGripperPos_mm = desiredGripperPos * 1000.0
        if desiredGripperPos_mm >= ysw.GRIPPER_POS_RANGE_LINEAR:
            desiredGripperPos_mm = ysw.GRIPPER_POS_RANGE_LINEAR - 8.0

        gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_L, operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE)
        gripper.current_based_position_control_linear(ysw.GRIPPER_ID_L, current_ampere_mag=0.02, linear_pos=desiredGripperPos_mm)
        time.sleep(1)
        gripper.disable(ysw.GRIPPER_ID_L)
            
        # =======================================================================
        # 3. Move to post-[R-to-L] posture
        # =======================================================================
        desired_pos = ysw.DEMO_RIGHT_TO_LEFT_POS_AFTER_260723  
        ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time * 0.3, control_hold_time=0.5)
        
        # =======================================================================
        # 4. Close Left Gripper & Open Right Gripper
        # =======================================================================
        gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_L, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
        gripper.write_current_ampere(ysw.GRIPPER_ID_L, grasp_current)
        time.sleep(1)
        gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE)
        gripper.current_based_position_control_linear(ysw.GRIPPER_ID_R, current_ampere_mag=0.02, linear_pos=desiredGripperPos_mm)
        time.sleep(1)
        gripper.disable(ysw.GRIPPER_ID_R)
        
        # =======================================================================
        # 5. Move to pre-[R-to-L] posture
        # =======================================================================
        desired_pos = ysw.DEMO_RIGHT_TO_LEFT_POS_BEFORE_260723  
        ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time * 0.3, control_hold_time=0.5)
        
        # =======================================================================
        # 6. Move to Yellow Box by using Left arm
        # =======================================================================
        pos_box = T_SC_target_for_box[:3,3]
        ori_box = ysw.rotMat_to_rpy(T_SC_target_for_box[:3,:3])       

        cartesian_position_targets_sample = [    
                                                {
                                                    "reference_link": "link_torso_5",
                                                    "target_link": "ee_finger_l1",
                                                    "pos": pos_box + np.array([0.0, 0.0, (np.mean(box_3Dsize)/1.5)]),
                                                    "rpy": ori_box                    
                                                },                 
                                            ]
        # [rv = "function"] form is Necessary when "get_return=False"
        rv = ysw.positionCtrl_cartesian_wholeBody(
            robot, 
            position_targets=cartesian_position_targets_sample, 
            joint_targets=None,
            minimum_time=position_control_minimum_time,
            control_hold_time=0.5,
            get_return=True
        )  
        
        # =======================================================================
        # 7. Open Left gripper to Lay
        # ======================================================================= 
        gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_L, operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE)
        gripper.current_based_position_control_linear(ysw.GRIPPER_ID_L, current_ampere_mag=0.02, linear_pos=desiredGripperPos_mm)
        time.sleep(1)
        gripper.disable(ysw.GRIPPER_ID_L)
        
    elif (grasp_arm == "right" and lay_arm == "right"):
        # =======================================================================
        # 1. Move to Yellow Box by using Right arm
        # =======================================================================
        pos_box = T_SC_target_for_box[:3,3]
        ori_box = ysw.rotMat_to_rpy(T_SC_target_for_box[:3,:3])       

        cartesian_position_targets_sample = [    
                                                {
                                                    "reference_link": "link_torso_5",
                                                    "target_link": "ee_finger_r1",
                                                    "pos": pos_box + np.array([0.0, 0.0, (np.mean(box_3Dsize)/1.5)]),
                                                    "rpy": ori_box                    
                                                },                 
                                            ]
        # [rv = "function"] form is Necessary when "get_return=False"
        rv = ysw.positionCtrl_cartesian_wholeBody(
            robot, 
            position_targets=cartesian_position_targets_sample, 
            joint_targets=None,
            minimum_time=position_control_minimum_time,
            control_hold_time=0.5,
            get_return=True
        )   
        
        # =======================================================================
        # 2. Open Right gripper to Lay
        # ======================================================================= 
        gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE)
        gripper.current_based_position_control_linear(ysw.GRIPPER_ID_R, current_ampere_mag=0.02, linear_pos=desiredGripperPos_mm)
        time.sleep(1)
        gripper.disable(ysw.GRIPPER_ID_R)
        
        
        
    # ======================================================================================================
    # Close ALL Grippers (End of 1-sequnece)
    # ======================================================================================================
    gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
    gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_L, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
    gripper.write_current_ampere(ysw.GRIPPER_ID_R, 0.04)
    gripper.write_current_ampere(ysw.GRIPPER_ID_L, 0.04)
    time.sleep(1.5)
    gripper.disable(ysw.GRIPPER_ID_R)
    gripper.disable(ysw.GRIPPER_ID_L)
    
    
    # ======================================================================================================
    # Counting the trial number
    # ======================================================================================================
    trial_idx = trial_idx + 1
    
    if (trial_idx == trial_num):
        desired_pos = ysw.DEMO_VISION_BASIC_POS_260723   
    ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time, control_hold_time=0.5)    
    