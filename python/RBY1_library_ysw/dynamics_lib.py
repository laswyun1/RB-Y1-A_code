# Dynamics Library for RBY1-A
# This Library contains robot kinematics and dynamics 
# This should be imported in "rby1_lib_ysw.py"
# 
# Usage example:
#       from dynamics_lib import *
#
# Copyright (c) 2026 Precision Mechatronics Laboratory, KAIST. All rights reserved.
# Author: Seungwon Yun


# =============================================================================
# Import
# =============================================================================
import rby1_sdk as rby
import robot_info as RI
import utils as UTIL
import logging

import numpy as np
from typing import Sequence


# =============================================================================
# Kinematics Functions
# =============================================================================
def rpy_to_rotMat(rpy: Sequence[float]=(0.0, 0.0, 0.0)) -> np.ndarray:
    """
    Build a 3x3 Rotation Matrix from RPY values
       RPY = ZYX Euler Angle
       
       [Rotation Sequence]
       RPY             : [Reference: Fixed coordinate]   X -> Y -> Z
       ZYX Eulear Angle: [Reference: Rotated coordinate] Z -> Y -> X  
       
    Args:
        rpy: sequence of 3 floats [roll, pitch, yaw], unit=[rad]

    Returns:
        3x3 Rotation Matrix.
    """
    roll, pitch, yaw = rpy
    
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
    return Rz @ Ry @ Rx  



def rotMat_to_rpy(rotMat: np.ndarray) -> np.ndarray:
    """
    Build 3x1 RPY from 3x3 Rotation Matrix
        RPY = ZYX Euler Angle

        [Rotation Sequence]
        RPY             : [Reference: Fixed coordinate]   X -> Y -> Z
        ZYX Eulear Angle: [Reference: Rotated coordinate] Z -> Y -> X  
    
    Args:
        rotMat: 3x3 Rotation Matrix 
    
    Returns: 
        3x1 RPY value: [roll, pitch, yaw], unit=[rad]
    """
    R = np.asarray(rotMat, dtype=float)

    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

    if sy > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])         # arctan2(y,x) = tan^-1(y/x), range: -180deg ~ +180deg
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock case
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return np.array([roll, pitch, yaw])



def rotMat_x(roll: float=0.0) -> np.ndarray:
    """Build a 3x3 Rotation Matrix for X-axis rotation

    Args:
        roll: float, unit=[rad]

    Returns:
        3x3 X-axis Rotation Matrix.
    """
    Rx = np.array([
        [1,      0,             0      ],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])
    return Rx   



def rotMat_y(pitch: float=0.0) -> np.ndarray:
    """Build a 3x3 Rotation Matrix for Y-axis rotation

    Args:
        pitch: float, unit=[rad]

    Returns:
        3x3 Y-axis Rotation Matrix.
    """
    Ry = np.array([
        [np.cos(pitch),  0, np.sin(pitch)],
        [       0,       1,       0      ],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    return Ry 


def rotMat_y(yaw: float=0.0) -> np.ndarray:
    """Build a 3x3 Rotation Matrix for Z-axis rotation

    Args:
        yaw: float, unit=[rad]

    Returns:
        3x3 Z-axis Rotation Matrix.
    """
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [     0,           0,       1]
    ])
    return Rz



def make_HT(rotMat: np.ndarray, translation: list[float]) -> np.ndarray:
    """Build a 4x4 homogeneous transform from rotation and translation.

    Args:
        rotMat     : 3x3 rotation matrix.
        translation: List of 3 floats [x, y, z].

    Returns:
        4x4 homogeneous transform.
    """
    T = np.eye(4)
    T[:3, :3] = rotMat
    T[:3, 3] = np.asarray(translation, dtype=float)
    return T



def get_FK(
    robot,
    reference_link: str,
    target_link: str
):
    """
    Get current pose of target_link with respect to reference_link.

    Args:
        robot:
            rby1_sdk robot object

        reference_link:
            ex) "base", "link_torso_5", "ee_right"

        target_link:
            ex) "ee_right", "ee_left", "link_torso_5"

    Returns:
        result:
            {
                "T": 4x4 homogeneous transformation matrix,
                "pos": np.array([x, y, z]),
                "rotMat": 3x3 rotation matrix,
                "rpy": np.array([roll, pitch, yaw])
            }

        Failed:
            None
    """
    UTIL.print_start_title(f"FK of '{target_link}' from '{reference_link}' Starts")

    # 1. Get robot model and dynamics model
    model = robot.model()
    dyn_robot = robot.get_dynamics()

    # 2. Check validity of link names
    if reference_link not in RI.ALL_LINK_NAMES:
        logging.error(f"reference link name {reference_link} is incorrect")
        return None
    if target_link not in RI.ALL_LINK_NAMES:
        logging.error(f"target link name {target_link} is incorrect")
        return None

    # 3. Make dynamics state (link list order is important)
    # 0-th: reference_link
    # 1-th: target_link
    dyn_state = dyn_robot.make_state([reference_link, target_link], model.robot_joint_names)

    # 4. Get current robot state
    robot_state = robot.get_state()
    q = np.asarray(robot_state.position, dtype=float)   # "robot_state.position" = current joint position(q)

    # Check full joint position values are acquired(24-DoF)
    if q.shape[0] != RI.DOF_TOTAL:
        logging.error(f"robot_state.position is not {RI.DOF_TOTAL}-DoF")
        return None

    # 5. Set q
    dyn_state.set_q(q)

    # vel and acc are not required for FK.
    # But, to maintain "dynamics state" format, set 0 all.
    dyn_state.set_qdot(np.zeros(RI.DOF_TOTAL))
    dyn_state.set_qddot(np.zeros(RI.DOF_TOTAL))

    # 6. Forward Kinematics
    dyn_robot.compute_forward_kinematics(dyn_state)

    # 7. Compute transformation
    # From (3) "make_state", the order is [reference_link, target_link], thus
    # 0: reference_link
    # 1: target_link
    try:
        T_ref_target = dyn_robot.compute_transformation(dyn_state, 0, 1)
    except Exception as e:
        logging.error(f"Failed to compute transformation: {e}")
        return None

    T_ref_target = np.asarray(T_ref_target, dtype=float)

    # 8. Extract position and orientation
    pos = T_ref_target[:3, 3]
    rotMat = T_ref_target[:3, :3]
    rpy = rotMat_to_rpy(rotMat)

    result = {
        "T": T_ref_target,
        "position": pos,
        "rotation": rotMat,
        "rpy": rpy
    }

    print("[Result]\n")
    print("T:")
    print(T_ref_target)
    print("\n")
    print(f"position [m]: {pos}")
    print(f"rpy [rad]: {rpy}")
    print(f"rpy [deg]: {rpy * RI.RAD2DEG}")

    UTIL.print_end_title(f"FK of '{target_link}' from '{reference_link}' Ends")
    return result




# =============================================================================
# Code
# =============================================================================
def make_joint_position_command(q_list, minimum_time=5.0, control_hold_time=0.5):
    """
    Base code for moving 22-DoF (Body: torso + right_arm + left_arm, Head)
    """
    return (rby.JointPositionCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time))
            .set_minimum_time(minimum_time)
            .set_position(q_list)
    )



def positionCtrl_joint_20dof(robot, posture, minimum_time=5.0, control_hold_time=0.5, get_return=True):
    """
    Move only 20-DoF Body segments (torso, right_arm, left_arm)
    Index: 2 ~ 21
    
    Args:
        posture:
            {
                "torso": np.array/list,      # 6 dof    [2, 3, 4, 5, 6, 7]
                "right_arm": np.array/list,  # 7 dof    [8, 9, 10, 11, 12, 13, 14]
                "left_arm": np.array/list    # 7 dof    [15, 16, 17, 18, 19, 20, 21]
            }
            
    rc: Robot Command
    rv: Return Value
    
    [Command Builder Structure]
    RobotCommandBuilder                             (.set_command)
    └── ComponentBasedCommandBuilder                (.set_body_command)
        └── BodyComponentBasedCommandBuilder        (.set_torso_command / .set_right_arm_command / .set_left_arm_command)
            ├── torso command                       (rby.JointPositionCommandBuilder())
            ├── right arm command                   (rby.JointPositionCommandBuilder())
            └── left arm command                    (rby.JointPositionCommandBuilder())
    """
    UTIL.print_start_title("20-DoF Joint Position Control Starts")
    
    component_cmd_builder = rby.ComponentBasedCommandBuilder()
    body_cmd_builder = rby.BodyComponentBasedCommandBuilder()
    has_body_command = False
    
    q_torso = posture.get("torso")
    q_right_arm = posture.get("right_arm")
    q_left_arm = posture.get("left_arm")
    
    #--- Body ---#
    if q_torso is not None:
        body_cmd_builder.set_torso_command(make_joint_position_command(q_torso, minimum_time, control_hold_time))
        has_body_command = True

    if q_right_arm is not None:
        body_cmd_builder.set_right_arm_command(make_joint_position_command(q_right_arm, minimum_time, control_hold_time))
        has_body_command = True

    if q_left_arm is not None:
        body_cmd_builder.set_left_arm_command(make_joint_position_command(q_left_arm, minimum_time, control_hold_time))
        has_body_command = True

    if has_body_command:
        component_cmd_builder.set_body_command(body_cmd_builder)
    else:
        logging.error("No command is applied")
        return False

    rc = rby.RobotCommandBuilder().set_command(component_cmd_builder)
    
    if (get_return == True):
        rv = robot.send_command(rc).get()
        
        if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
            logging.error("Failed to conduct [move_position_22dof].")
            return False
        UTIL.print_end_title("20-DoF Joint Position Control Ends")
        return True
    else:
        rv = robot.send_command(rc)
        UTIL.print_end_title("20-DoF Joint Position Control Ends")
        return rv




def positionCtrl_joint_22dof(robot, posture, minimum_time=5.0, control_hold_time=0.5, get_return=True):
    """
    Move only 22-DoF (Body + Head) segments (torso, right_arm, left_arm, neck)
    Index: 2 ~ 21
    
    posture:
        {
            "torso": np.array/list,      # 6 dof    [2, 3, 4, 5, 6, 7]
            "right_arm": np.array/list,  # 7 dof    [8, 9, 10, 11, 12, 13, 14]
            "left_arm": np.array/list    # 7 dof    [15, 16, 17, 18, 19, 20, 21]
            "head": np.array/list        # 2 dof    [22, 23]   (0: yaw, 1: pitch)
        }
        
    rc: Robot Command
    rv: Return Value
    
    [Command Builder Structure]
    RobotCommandBuilder                             (.set_command)
    └── ComponentBasedCommandBuilder                (.set_body_command / .set_head_command)
        └── BodyComponentBasedCommandBuilder        (.set_torso_command / .set_right_arm_command / .set_left_arm_command)
            ├── torso command                       (rby.JointPositionCommandBuilder())
            ├── right arm command                   (rby.JointPositionCommandBuilder())
            └── left arm command                    (rby.JointPositionCommandBuilder())
    """
    UTIL.print_start_title("22-DoF Joint Position Control Starts")
    component_cmd_builder = rby.ComponentBasedCommandBuilder()
    body_cmd_builder = rby.BodyComponentBasedCommandBuilder()
    has_body_command = False
    
    q_torso = posture.get("torso")
    q_right_arm = posture.get("right_arm")
    q_left_arm = posture.get("left_arm")
    q_head = posture.get("head")
    
    #--- Body ---#
    if q_torso is not None:
        body_cmd_builder.set_torso_command(make_joint_position_command(q_torso, minimum_time, control_hold_time))
        has_body_command = True

    if q_right_arm is not None:
        body_cmd_builder.set_right_arm_command(make_joint_position_command(q_right_arm, minimum_time, control_hold_time))
        has_body_command = True

    if q_left_arm is not None:
        body_cmd_builder.set_left_arm_command(make_joint_position_command(q_left_arm, minimum_time, control_hold_time))
        has_body_command = True

    if has_body_command:
        component_cmd_builder.set_body_command(body_cmd_builder)

    #--- Head ---#
    if q_head is not None:
        component_cmd_builder.set_head_command(make_joint_position_command(q_head, minimum_time))

    if not has_body_command and q_head is None:
        logging.error("No command is applied")
        return False

    rc = rby.RobotCommandBuilder().set_command(component_cmd_builder)
    
    if (get_return == True):
        rv = robot.send_command(rc).get()
        
        if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
            logging.error("Failed to conduct [move_position_22dof].")
            return False
        UTIL.print_end_title("22-DoF Joint Position Control Ends")
        return True
    else:
        rv = robot.send_command(rc)
        UTIL.print_end_title("22-DoF Joint Position Control Ends")
        return rv



def positionCtrl_joint_head(robot, q_head, minimum_time=2.0, control_hold_time=0.5, get_return=True):
    """
    Move only 2-DoF Head segments
    Index: 22 ~ 23
    
    q_head: np.array/list  # 2 dof        [head_0: yaw, head_1: pitch]
    """
    UTIL.print_start_title("2-DoF Head Joint Position Control Starts")
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_head_command(
            rby.JointPositionCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time))
            .set_minimum_time(minimum_time)
            .set_position(q_head)
        )
    )    
    if (get_return == True):
        rv = robot.send_command(rc).get()
        UTIL.print_end_title("2-DoF Head Joint Position Control Ends")
        return rc.finish_code == rby.RobotCommandFeedback.FinishCode.Ok
    else:
        rv = robot.send_command(rc)
        UTIL.print_end_title("2-DoF Head Joint Position Control Ends")
        return rv



def positionCtrl_cartesian_fixed(
    robot, 
    cartesian_targets: list[dict], 
    joint_targets: dict | None = None, 
    minimum_time: float = RI.MINIMUM_TIME,
    linear_velocity_limit: float = RI.LINEAR_VELOCITY_LIMIT,
    angular_velocity_limit: float = RI.ANGULAR_VELOCITY_LIMIT,
    acceleration_limit: float = RI.ACCELERATION_LIMIT,
    stop_position_tracking_error: float = RI.STOP_POSITION_TRACKING_ERROR,
    stop_orientation_tracking_error: float = RI.STOP_ORIENTATION_TRACKING_ERROR,
    control_hold_time: float | None = None,
    get_return: bool = True
):
    """
    Cartesian Position Control (Fixed version)

    Move target links with respect to reference links.
    Fix other components

    For example, if "moving_component" is "right_arm",
    then just 7-DoF "right_arm" joints move while fixing other joints.
    
    Caution!!:
        "target_link" should be matched with "component"  

    Args:
        cartesian_targets:
            [
                {
                    "moving_component": "right_arm",            # "torso", "right_arm", "left_arm" 
                    "reference_link": "base",
                    "target_link": "ee_right",
                    "pos": [x, y, z],
                    "rpy": [roll, pitch, yaw],
                },
                {
                    "moving_component": "left_arm",
                    "reference_link": "base",
                    "target_link": "ee_left",
                    "pos": [x, y, z],
                    "rpy": [roll, pitch, yaw],
                },
            ]

        joint_targets:
            {
                "right_arm_1": -np.pi / 3,
                "left_arm_1": np.pi / 3,
            }
    """

    UTIL.print_start_title("Cartesian Position Control(Fixed) Starts")
    
    # 1. Argument validity check
    if cartesian_targets is None or len(cartesian_targets) == 0:
        logging.error("No Cartesian target is given")
        return False

    # 2. Make Body Component Command Builder
    body_cmd_builder = rby.BodyComponentBasedCommandBuilder()

    # 3. Make each Cartesian Command Builder
    torso_cmd_builder = None
    right_arm_cmd_builder = None
    left_arm_cmd_builder = None

    has_torso_command = False
    has_right_arm_command = False
    has_left_arm_command = False

    # 4. Add Cartesian targets
    for i, cartesian_target in enumerate(cartesian_targets):
        moving_component = cartesian_target.get("moving_component")
        reference_link = cartesian_target.get("reference_link")
        target_link = cartesian_target.get("target_link")
        pos = cartesian_target.get("pos")
        rpy = cartesian_target.get("rpy")

        if moving_component is None:
            logging.error(f"cartesian_target {i} has no component")
            return False

        if moving_component not in ["torso", "right_arm", "left_arm"]:
            logging.error(
                f"cartesian_target {i}['component'] must be "
                "'torso', 'right_arm', or 'left_arm'"
            )
            return False

        if reference_link is None:
            logging.error(f"cartesian_target {i} has no reference_link")
            return False

        if target_link is None:
            logging.error(f"cartesian_target {i} has no target_link")
            return False

        if rpy is None or len(rpy) != 3:
            logging.error(f"cartesian_target {i}['rpy'] must have 3 elements")
            return False

        if pos is None or len(pos) != 3:
            logging.error(f"cartesian_target {i}['pos'] must have 3 elements")
            return False

        # 5. Make Homogeneous Transform Matrix
        T = make_HT(rpy_to_rotMat(rpy), pos)

        # 6. Select moving_component builder
        if moving_component == "torso":
            if torso_cmd_builder is None:
                torso_cmd_builder = rby.CartesianCommandBuilder()

            torso_cmd_builder.add_target(
                reference_link,
                target_link,
                T,
                linear_velocity_limit,
                angular_velocity_limit,
                acceleration_limit
            )
            has_torso_command = True

        elif moving_component == "right_arm":
            if right_arm_cmd_builder is None:
                right_arm_cmd_builder = rby.CartesianCommandBuilder()

            right_arm_cmd_builder.add_target(
                reference_link,
                target_link,
                T,
                linear_velocity_limit,
                angular_velocity_limit,
                acceleration_limit
            )
            has_right_arm_command = True

        elif moving_component == "left_arm":
            if left_arm_cmd_builder is None:
                left_arm_cmd_builder = rby.CartesianCommandBuilder()

            left_arm_cmd_builder.add_target(
                reference_link,
                target_link,
                T,
                linear_velocity_limit,
                angular_velocity_limit,
                acceleration_limit
            )
            has_left_arm_command = True

        logging.info(
            f"Add (Fixed) Cartesian target {i}: "
            f"component='{moving_component}', '{reference_link}'->'{target_link}', "
            f"pos={pos}, rpy={rpy}"         # Relative position and orientation
        )

    # 7. Add joint position targets if needed
    if joint_targets is not None:
        for joint_name, joint_pos in joint_targets.items():

            if joint_name.startswith("torso"):
                if torso_cmd_builder is None:
                    torso_cmd_builder = rby.CartesianCommandBuilder()

                torso_cmd_builder.add_joint_position_target(
                    joint_name,
                    float(joint_pos)
                )
                has_torso_command = True
                
            elif joint_name.startswith("right_arm"):
                if right_arm_cmd_builder is None:
                    right_arm_cmd_builder = rby.CartesianCommandBuilder()

                right_arm_cmd_builder.add_joint_position_target(
                    joint_name,
                    float(joint_pos)
                )
                has_right_arm_command = True

            elif joint_name.startswith("left_arm"):
                if left_arm_cmd_builder is None:
                    left_arm_cmd_builder = rby.CartesianCommandBuilder()

                left_arm_cmd_builder.add_joint_position_target(
                    joint_name,
                    float(joint_pos)
                )
                has_left_arm_command = True

            else:
                logging.error(f"Unknown joint name: '{joint_name}'")
                return False

            logging.info(f"Add joint position target: '{joint_name}' = {joint_pos}")


    # 8. Set options for each component command
    def set_cartesian_common_options(cmd_builder):
        cmd_builder.set_minimum_time(minimum_time)
        cmd_builder.set_stop_position_tracking_error(stop_position_tracking_error)
        cmd_builder.set_stop_orientation_tracking_error(stop_orientation_tracking_error)

        if control_hold_time is not None:
            cmd_builder.set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time)
            )
        return cmd_builder

    if has_torso_command:
        torso_cmd_builder = set_cartesian_common_options(torso_cmd_builder)
        body_cmd_builder.set_torso_command(torso_cmd_builder)

    if has_right_arm_command:
        right_arm_cmd_builder = set_cartesian_common_options(right_arm_cmd_builder)
        body_cmd_builder.set_right_arm_command(right_arm_cmd_builder)

    if has_left_arm_command:
        left_arm_cmd_builder = set_cartesian_common_options(left_arm_cmd_builder)
        body_cmd_builder.set_left_arm_command(left_arm_cmd_builder)

    if not has_torso_command and not has_right_arm_command and not has_left_arm_command:
        logging.error("No Cartesian command is applied")
        return False

    # 9. Build robot command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            body_cmd_builder
        )
    )

    # 10. Send command
    if get_return:
        rv = robot.send_command(rc).get()

        if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
            logging.error(
                f"Failed to conduct Cartesian Position Control. "
                f"finish_code = {rv.finish_code}"
            )
            return False

        UTIL.print_end_title("Cartesian Position Control(Fixed) Ends")
        return True

    else:
        rv = robot.send_command(rc)
        UTIL.print_end_title("Cartesian Position Control(Fixed) Ends")
        return rv
    


def positionCtrl_cartesian_NotFixed(
    robot, 
    cartesian_targets: list[dict], 
    joint_targets: dict | None=None, 
    minimum_time: float = RI.MINIMUM_TIME,
    linear_velocity_limit: float = RI.LINEAR_VELOCITY_LIMIT,
    angular_velocity_limit: float = RI.ANGULAR_VELOCITY_LIMIT,
    acceleration_limit: float = RI.ACCELERATION_LIMIT,
    stop_position_tracking_error: float = RI.STOP_POSITION_TRACKING_ERROR,
    stop_orientation_tracking_error: float = RI.STOP_ORIENTATION_TRACKING_ERROR,
    control_hold_time: float | None = None,
    get_return=True
):
    """
    Cartesian(Relative) Position Control
    Move "target link" coordinate with respect to "reference link" coordinate
    
    Args:
        cartesian_targets:  list of "reference link, target link, relative RPY, relative Position" 
            [
                {
                    "reference_link": "base",
                    "target_link": "ee_right",
                    "rpy": [roll, pitch, yaw],
                    "pos": [x, y, z],
                },
                {
                    "reference_link": "base",
                    "target_link": "ee_left",
                    "rpy": [roll, pitch, yaw],
                    "pos": [x, y, z],
                }, ...
            ]
        
        joint_targets:  Additional constraints for certain joint to reduce redundancy
            {
                "right_arm_1": -np.pi / 3,
                "left_arm_1": np.pi / 3,
            }
            
        Order of these 2 arguments should be matched !!
    """
    UTIL.print_start_title("Cartesian Position Control Starts")
    
    # 1. Argument Validity Check
    if cartesian_targets is None or len(cartesian_targets) == 0:
        logging.error("No Cartesian target is given")
        return False
    
    # 2. Make Cartesian Command Builder
    cartesian_cmd_builder = rby.CartesianCommandBuilder()
    
    for i, cartesian_target in enumerate(cartesian_targets):
        reference_link = cartesian_target.get("reference_link")
        target_link    = cartesian_target.get("target_link")
        rpy            = cartesian_target.get("rpy")
        pos            = cartesian_target.get("pos")

        if reference_link is None:
            logging.error(f"cartesian target[{i}] has no reference_link")
            return False

        if target_link is None:
            logging.error(f"cartesian target[{i}] has no target_link")
            return False

        if rpy is None or len(rpy) != 3:
            logging.error(f"cartesian target[{i}]['rpy'] must have 3 elements")
            return False

        if pos is None or len(pos) != 3:
            logging.error(f"cartesian target[{i}]['pos'] must have 3 elements")
            return False
        
        # 3. Make Homogeneous Transform Matrix
        T = make_HT(rpy_to_rotMat(rpy), pos)
        
        # 4. Add Cartesian target
        cartesian_cmd_builder.add_target(
            reference_link,
            target_link,
            T,
            linear_velocity_limit,
            angular_velocity_limit,
            acceleration_limit
        )

        logging.info(
            f"Add Cartesian target [{i}]: "
            f"{reference_link} -> {target_link}, rpy={rpy}, pos={pos}"
        )
        
    # 5. Add joint position targets if needed
    if joint_targets is not None:
        for joint_name, joint_pos in joint_targets.items():
            cartesian_cmd_builder.add_joint_position_target(joint_name, float(joint_pos))
            logging.info(f"Add joint position target: {joint_name} = {joint_pos}")
                
    # 6. Set common options
    cartesian_cmd_builder.set_minimum_time(minimum_time)
    cartesian_cmd_builder.set_stop_position_tracking_error(stop_position_tracking_error)
    cartesian_cmd_builder.set_stop_orientation_tracking_error(stop_orientation_tracking_error)

    if control_hold_time is not None:
        cartesian_cmd_builder.set_command_header(
            rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time)
        )

    # 7. Build robot command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            cartesian_cmd_builder
        )
    )

    # 8. Send command
    if get_return:
        rv = robot.send_command(rc).get()

        if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
            logging.error("Failed to conduct Cartesian Position Control")
            return False

        UTIL.print_end_title("Cartesian Position Control Ends")
        return True
    else:
        rv = robot.send_command(rc)
        UTIL.print_end_title("Cartesian Position Control Ends")
        return rv




    