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
    Base code for Joint Position Control 22-DoF (Body: torso + right_arm + left_arm, Head)
    """
    return (rby.JointPositionCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time))
            .set_minimum_time(minimum_time)
            .set_position(q_list)
    )



def make_joint_impedance_command(
        q_list,
        stiffness=RI.DEFAULT_STIFFNESS,
        damping_ratio=RI.DEFAULT_DAMPING_RATIO, 
        torque_limit=RI.DEFAULT_TORQUE_LIMIT,
        minimum_time=5.0, 
        control_hold_time=0.5):
    """
    Base code for Joint Impedance Control 20-DoF (Body: torso + right_arm + left_arm)
    """
    return (rby.JointImpedanceControlCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time))
            .set_position(q_list)
            .set_stiffness(stiffness)
            .set_damping_ratio(damping_ratio)
            .set_minimum_time(minimum_time)
            .set_torque_limit(torque_limit)
    )



def positionCtrl_joint_20dof(robot, posture, minimum_time=5.0, control_hold_time=0.5, get_return=True):
    """
    Move only 20-DoF Body segments (torso, right_arm, left_arm)
    Index: 2 ~ 21

    When use this function:
        [hdlr = "function"] form is Necessary when "get_return=False"
    
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
    
    if get_return:
        rv = robot.send_command(rc).get()
        
        if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
            logging.error("Failed to conduct [move_position_22dof].")
            return False
        UTIL.print_end_title("20-DoF Joint Position Control Ends")
        return True
    else:
        handler = robot.send_command(rc)
        UTIL.print_end_title("20-DoF Joint Position Control Ends")
        return handler




def positionCtrl_joint_22dof(robot, posture, minimum_time=5.0, control_hold_time=0.5, get_return=True):
    """
    Move only 22-DoF (Body + Head) segments (torso, right_arm, left_arm, neck)
    Index: 2 ~ 21

    When use this function:
        [hdlr = "function"] form is Necessary when "get_return=False"
    
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
    
    q_torso = posture.get(RI.TORSO)
    q_right_arm = posture.get(RI.RIGHT_ARM)
    q_left_arm = posture.get(RI.LEFT_ARM)
    q_head = posture.get(RI.HEAD)
    
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
    
    if get_return:
        rv = robot.send_command(rc).get()
        
        if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
            logging.error("Failed to conduct [move_position_22dof].")
            return False
        UTIL.print_end_title("22-DoF Joint Position Control Ends")
        return True
    else:
        handler = robot.send_command(rc)
        UTIL.print_end_title("22-DoF Joint Position Control Ends")
        return handler



def positionCtrl_joint_head(robot, q_head, minimum_time=2.0, control_hold_time=0.5, get_return=True):
    """
    Move only 2-DoF Head segments
    Index: 22 ~ 23

    When use this function:
        [hdlr = "function"] form is Necessary when "get_return=False"
    
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
    if get_return:
        rv = robot.send_command(rc).get()
        UTIL.print_end_title("2-DoF Head Joint Position Control Ends")
        return rc.finish_code == rby.RobotCommandFeedback.FinishCode.Ok
    else:
        handler = robot.send_command(rc)
        UTIL.print_end_title("2-DoF Head Joint Position Control Ends")
        return handler



def positionCtrl_cartesian_componentWise(
    robot, 
    position_targets: list[dict], 
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
    Cartesian Position Control (Component-wise version)

    Move target links with respect to reference links.
    Fix other components (torso[6-DoF] / right arm[7-DoF] / left arm[7-DoF])

    For example, if "moving_component" is "right_arm",
    then just 7-DoF "right_arm" joints move while fixing other joints.

    "joint_targets" are additional constraints to keep
    
    Caution!!:
        "target_link" should be matched with "moving_component"  

    When use this function:
        [hdlr = "function"] form is Necessary when "get_return=False"    
        
    Args:
        position_targets:
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

    UTIL.print_start_title("Cartesian Position Control(Component-wise) Starts")
    
    # 1. Argument validity check
    if position_targets is None or len(position_targets) == 0:
        logging.error("No Cartesian Position target is given")
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
    for i, position_target in enumerate(position_targets):
        moving_component = position_target.get("moving_component")
        reference_link = position_target.get("reference_link")
        target_link = position_target.get("target_link")
        pos = position_target.get("pos")
        rpy = position_target.get("rpy")

        if moving_component is None:
            logging.error(f"position_target {i} has no 'moving_component'")
            return False

        if moving_component not in RI.BODY_COMPONENT_NAME_LIST:
            logging.error(
                f"position_target {i}['component'] must be "
                "'torso', 'right_arm', or 'left_arm'"
            )
            return False

        if reference_link is None:
            logging.error(f"position_target {i} has no 'reference_link'")
            return False

        if target_link is None:
            logging.error(f"position_target {i} has no 'target_link'")
            return False
        
        if pos is None or len(pos) != 3:
            logging.error(f"position_target {i}['pos'] must have 3 elements")
            return False
        
        if rpy is None or len(rpy) != 3:
            logging.error(f"position_target {i}['rpy'] must have 3 elements")
            return False

        # 5. Make Homogeneous Transform Matrix
        T = make_HT(rpy_to_rotMat(rpy), pos)

        # 6. Select moving_component builder
        if moving_component == RI.TORSO:
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

        elif moving_component == RI.RIGHT_ARM:
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

        elif moving_component == RI.LEFT_ARM:
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
            f"Add (Component-wise) Position target {i}: "
            f"component='{moving_component}', '{reference_link}'->'{target_link}', "
            f"pos={pos}, rpy={rpy}"         # Relative position and orientation
        )

    # 7. Add joint position targets if needed
    if joint_targets is not None:
        for joint_name, joint_pos in joint_targets.items():

            if joint_name.startswith(RI.TORSO):
                if torso_cmd_builder is None:
                    torso_cmd_builder = rby.CartesianCommandBuilder()

                torso_cmd_builder.add_joint_position_target(
                    joint_name,
                    float(joint_pos)
                )
                has_torso_command = True
                
            elif joint_name.startswith(RI.RIGHT_ARM):
                if right_arm_cmd_builder is None:
                    right_arm_cmd_builder = rby.CartesianCommandBuilder()

                right_arm_cmd_builder.add_joint_position_target(
                    joint_name,
                    float(joint_pos)
                )
                has_right_arm_command = True

            elif joint_name.startswith(RI.LEFT_ARM):
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
                f"Failed to conduct Cartesian Position Control(Component-wise). "
                f"finish_code = {rv.finish_code}"
            )
            return False

        UTIL.print_end_title("Cartesian Position Control(Component-wise) Ends")
        return True

    else:
        handler = robot.send_command(rc)
        UTIL.print_end_title("Cartesian Position Control(Component-wise) Ends")
        return handler
    


def positionCtrl_cartesian_wholeBody(
    robot, 
    position_targets: list[dict], 
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
    Cartesian Position Control (Whole-Body version)

    Move target links with respect to reference links.
    Do not Fix other components (Using Whole-Body joints)

    For example, if reference link is "base" and target link is "ee_left",
    The robot can move ALL parts(Not only use "left_arm" 7-DoF) to achieve the target

    "joint_targets" are additional constraints to keep

    When use this function: 
        [hdlr = "function"] form is Necessary when "get_return=False"

    Args:
        position_targets:
            [
                {
                    "reference_link": "base",
                    "target_link": "ee_right",
                    "pos": [x, y, z],
                    "rpy": [roll, pitch, yaw],
                },
                {
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
    UTIL.print_start_title("Cartesian Position Control(Whole-Body) Starts")
    
    # 1. Argument validity check
    if position_targets is None or len(position_targets) == 0:
        logging.error("No Position target is given")
        return False

    # 2. Make Cartesian Command Builder
    cartesian_cmd_builder = rby.CartesianCommandBuilder()

    # 3. Add Cartesian targets
    for i, position_target in enumerate(position_targets):
        reference_link = position_target.get("reference_link")
        target_link = position_target.get("target_link")
        pos = position_target.get("pos")
        rpy = position_target.get("rpy")

        if reference_link is None:
            logging.error(f"position_target {i} has no 'reference_link'")
            return False

        if target_link is None:
            logging.error(f"position_target {i} has no 'target_link'")
            return False
        
        if pos is None or len(pos) != 3:
            logging.error(f"position_target {i}['pos'] must have 3 elements")
            return False

        if rpy is None or len(rpy) != 3:
            logging.error(f"position_target {i}['rpy'] must have 3 elements")
            return False

        # 4. Make Homogeneous Transform Matrix
        T = make_HT(rpy_to_rotMat(rpy), pos)

        # 5. Add Cartesian target
        cartesian_cmd_builder.add_target(
            reference_link,
            target_link,
            T,
            linear_velocity_limit,
            angular_velocity_limit,
            acceleration_limit
        )

        logging.info(
            f"Add (Whole-Body) Cartesian target {i}: "
            f"'{reference_link}'->'{target_link}', "
            f"pos={pos}, rpy={rpy}"         # Relative position and orientation
        )

    # 6. Add joint position targets if needed
    if joint_targets is not None:
        for joint_name, joint_pos in joint_targets.items():
            cartesian_cmd_builder.add_joint_position_target(joint_name, float(joint_pos))
            logging.info(f"Add joint position target: {joint_name} = {joint_pos}")

    # 7. Set options 
    cartesian_cmd_builder.set_minimum_time(minimum_time)
    cartesian_cmd_builder.set_stop_position_tracking_error(stop_position_tracking_error)
    cartesian_cmd_builder.set_stop_orientation_tracking_error(stop_orientation_tracking_error)

    if control_hold_time is not None:
        cartesian_cmd_builder.set_command_header(
            rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time)
        )

    # 8. Build robot command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            cartesian_cmd_builder
        )
    )

    # 9. Send command
    if get_return:
        rv = robot.send_command(rc).get()

        if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
            logging.error(
                f"Failed to conduct Cartesian Position Control(Whole-Body). "
                f"finish_code = {rv.finish_code}"
            )
            return False

        UTIL.print_end_title("Cartesian Position Control(Whole-Body) Ends")
        return True

    else:
        handler = robot.send_command(rc)
        UTIL.print_end_title("Cartesian Position Control(Whole-Body) Ends")
        return handler



def impedanceCtrl_cartesian_componentWise(
    robot, 
    impedance_targets: list[dict], 
    default_translation_weight: list[float] = RI.DEFAULT_TRANSLATION_WEIGHT,
    default_rotation_weight: list[float] = RI.DEFAULT_ROTATION_WEIGHT,
    default_damping_ratio: float = RI.DEFAULT_DAMPING_RATIO,
    control_hold_time: float = RI.DEFAULT_CONTROL_HOLD_TIME,     # Important!! 
    get_return=True
):
    """
    Cartesian Impedance Control (Component-wise version)

    Move target links with respect to reference links using [impedance control].
    Fix other components (torso[6-DoF] / right_arm[7-DoF] / left_arm[7-DoF]).

    For example, if "moving_component" is "right_arm",
    then only 7-DoF right_arm joints move while other body components are fixed.

    When use this function: 
        [hdlr = "function"] form is Necessary when "get_return=False"    

    Args:
        impedance_targets:
            [
                {
                    "moving_component": "torso",                # "torso", "right_arm", "left_arm"
                    "reference_link": "base",
                    "target_link": "link_torso_5",
                    "pos": [x, y, z],
                    "rpy": [roll, pitch, yaw],

                    # optional
                    "translation_weight": [1000, 1000, 1000],
                    "rotation_weight": [100, 100, 100],
                    "damping_ratio": 0.85,
                    "control_hold_time": 3.0,
                },
                {
                    "moving_component": "right_arm",
                    "reference_link": "link_torso_5",
                    "target_link": "ee_right",
                    "pos": [x, y, z],
                    "rpy": [roll, pitch, yaw],

                    # optional
                    "translation_weight": [1000, 1000, 1000],
                    "rotation_weight": [50, 50, 50],
                    "damping_ratio": 0.85,
                },
            ]

        translation_weight:
            Higher value means larger stiffness

        rotation_weight:
            Higher value means larger stiffness

        damping_ratio:
            Higher value means larger damping

    
    [CMD structure]
        RobotCommandBuilder
        └── ComponentBasedCommandBuilder
            └── BodyComponentBasedCommandBuilder
                ├── torso_command
                ├── right_arm_command
                └── left_arm_command
    """
    UTIL.print_start_title("Cartesian Impedance Control(Component-wise) Starts")
    
    # 1. Argument validity check
    if impedance_targets is None or len(impedance_targets) == 0:
        logging.error("No impedance target is given")
        return False

    # 2. Make Body Component Command Builder
    body_cmd_builder = rby.BodyComponentBasedCommandBuilder()

    torso_cmd_builder = None
    right_arm_cmd_builder = None
    left_arm_cmd_builder = None

    has_torso_command = False
    has_right_arm_command = False
    has_left_arm_command = False

    # 3. Add impedance targets
    for i, impedance_target in enumerate(impedance_targets):
        moving_component = impedance_target.get("moving_component")
        reference_link = impedance_target.get("reference_link")
        target_link = impedance_target.get("target_link")
        pos = impedance_target.get("pos")
        rpy = impedance_target.get("rpy")

        translation_weight = impedance_target.get(
            "translation_weight",
            default_translation_weight
        )
        rotation_weight = impedance_target.get(
            "rotation_weight",
            default_rotation_weight
        )
        damping_ratio = impedance_target.get(
            "damping_ratio",
            default_damping_ratio
        )
        target_control_hold_time = impedance_target.get(
            "control_hold_time",
            control_hold_time
        )

        if moving_component is None:
            logging.error(f"impedance_target {i} has no 'moving_component'")
            return False

        if moving_component not in RI.BODY_COMPONENT_NAME_LIST:
            logging.error(
                f"impedance_target {i}['moving_component'] must be "
                "'torso', 'right_arm', or 'left_arm'"
            )
            return False

        if reference_link is None:
            logging.error(f"impedance_target {i} has no 'reference_link'")
            return False

        if target_link is None:
            logging.error(f"impedance_target {i} has no 'target_link'")
            return False

        if pos is None or len(pos) != 3:
            logging.error(f"impedance_target {i}['pos'] must have 3 elements")
            return False

        if rpy is None or len(rpy) != 3:
            logging.error(f"impedance_target {i}['rpy'] must have 3 elements")
            return False

        if translation_weight is None or len(translation_weight) != 3:
            logging.error(
                f"impedance_target {i}['translation_weight'] must have 3 elements"
            )
            return False

        if rotation_weight is None or len(rotation_weight) != 3:
            logging.error(
                f"impedance_target {i}['rotation_weight'] must have 3 elements"
            )
            return False

        # 4. Make Homogeneous Transform Matrix
        T = make_HT(rpy_to_rotMat(rpy), pos)

        # 5. Make Impedance Control Command Builder
        impedance_cmd_builder = (
            rby.ImpedanceControlCommandBuilder()
            .set_reference_link_name(reference_link)
            .set_link_name(target_link)
            .set_translation_weight(translation_weight)     # [x,y,z] based on reference_link frame
            .set_rotation_weight(rotation_weight)           # [x,y,z] based on reference_link frame
            .set_transformation(T)
        )

        # 6. Set options (ImpedanceControlCommandBuilder' object has no attribute 'set_minimum_time')
        if control_hold_time is not None: # [Important] If 'control_hold_time' is 0 or None, the robot is not moving
            impedance_cmd_builder.set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(target_control_hold_time)
            )

        if damping_ratio is not None:
            impedance_cmd_builder.set_damping_ratio(float(damping_ratio))

        # 7. Select moving_component command
        if moving_component == RI.TORSO:
            torso_cmd_builder = impedance_cmd_builder
            has_torso_command = True

        elif moving_component == RI.RIGHT_ARM:
            right_arm_cmd_builder = impedance_cmd_builder
            has_right_arm_command = True

        elif moving_component == RI.LEFT_ARM:
            left_arm_cmd_builder = impedance_cmd_builder
            has_left_arm_command = True

        logging.info(
            f"Add (Component-wise) Impedance target {i}: "
            f"moving_component='{moving_component}', "
            f"'{reference_link}'->'{target_link}', "
            f"pos={pos}, rpy={rpy}, "
            f"translation_weight={translation_weight}, "
            f"rotation_weight={rotation_weight}, "
            f"damping_ratio={damping_ratio}, "
            f"control_hold_time={target_control_hold_time}"
        )

    # 8. Apply component commands
    if has_torso_command:
        body_cmd_builder.set_torso_command(torso_cmd_builder)

    if has_right_arm_command:
        body_cmd_builder.set_right_arm_command(right_arm_cmd_builder)

    if has_left_arm_command:
        body_cmd_builder.set_left_arm_command(left_arm_cmd_builder)

    if not has_torso_command and not has_right_arm_command and not has_left_arm_command:
        logging.error("No impedance command is applied")
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
                f"Failed to conduct Cartesian Impedance Control(Component-wise). "
                f"finish_code = {rv.finish_code}"
            )
            return False

        UTIL.print_end_title("Cartesian Impedance Control(Component-wise) Ends")
        return True

    else:
        handler = robot.send_command(rc)
        UTIL.print_end_title("Cartesian Impedance Control(Component-wise) Ends")
        return handler
    


def mixedCtrl_PositionAndImpedance_cartesian_componentWise(
    robot,
    position_targets: list[dict] | None = None,
    impedance_targets: list[dict] | None = None,
    minimum_time_PC: float = RI.MINIMUM_TIME,
    linear_velocity_limit_PC: float = RI.LINEAR_VELOCITY_LIMIT,
    angular_velocity_limit_PC: float = RI.ANGULAR_VELOCITY_LIMIT,
    acceleration_limit_PC: float = RI.ACCELERATION_LIMIT,
    stop_position_tracking_error_PC: float = RI.STOP_POSITION_TRACKING_ERROR,
    stop_orientation_tracking_error_PC: float = RI.STOP_ORIENTATION_TRACKING_ERROR,
    default_translation_weight_IC: list[float] = RI.DEFAULT_TRANSLATION_WEIGHT,
    default_rotation_weight_IC: list[float] = RI.DEFAULT_ROTATION_WEIGHT,
    default_damping_ratio_IC: float = RI.DEFAULT_DAMPING_RATIO,
    control_hold_time_PC: float | None = None,           
    control_hold_time_IC: float = RI.DEFAULT_CONTROL_HOLD_TIME,     # Important!! 
    get_return: bool = True,
):
    """
    [Position + Impedance] Control (Component-wise version)

    This function can mix:
        - target1: Cartesian position control
        - target2: Cartesian impedance control

    Example:
        right_arm:
            Position control
            Move ee_right with respect to base.

        left_arm:
            Impedance control
            Keep ee_left relative to ee_right.

    When use this function: 
        [hdlr = "function"] form is Necessary when "get_return=False"

    Args:
        position_targets:
            [
                {
                    "moving_component": "right_arm",        # "torso", "right_arm", "left_arm"
                    "reference_link": "base",
                    "target_link": "ee_right",
                    "pos": [x, y, z],
                    "rpy": [roll, pitch, yaw],

                    # optional
                    "minimum_time": 3.0,
                    "control_hold_time": 0.5,
                    "joint_targets": {
                        "right_arm_1": -np.pi / 3,
                    },
                },
            ]

        impedance_targets:
            [
                {
                    "moving_component": "left_arm",         # "torso", "right_arm", "left_arm"
                    "reference_link": "ee_right",   
                    "target_link": "ee_left",
                    "pos": [x, y, z],
                    "rpy": [roll, pitch, yaw],

                    # optional
                    "translation_weight": [1000, 1000, 1000],
                    "rotation_weight": [50, 50, 50],
                    "damping_ratio": 0.85,
                    "control_hold_time": 3.0,
                },
            ]

    [CMD structure]
        RobotCommandBuilder
        └── ComponentBasedCommandBuilder
            └── BodyComponentBasedCommandBuilder
                ├── torso_command
                ├── right_arm_command
                └── left_arm_command
    """

    UTIL.print_start_title("Cartesian (Position + Impedance) Control Starts")

    if (position_targets is None or len(position_targets) == 0) and \
       (impedance_targets is None or len(impedance_targets) == 0):
        logging.error("No position or impedance target is given")
        return False

    body_cmd_builder = rby.BodyComponentBasedCommandBuilder()

    torso_cmd_builder = None
    right_arm_cmd_builder = None
    left_arm_cmd_builder = None

    has_torso_command = False
    has_right_arm_command = False
    has_left_arm_command = False

    valid_components = RI.BODY_COMPONENT_NAME_LIST

    def set_component_command(moving_component: str, cmd_builder):
        nonlocal torso_cmd_builder
        nonlocal right_arm_cmd_builder
        nonlocal left_arm_cmd_builder
        nonlocal has_torso_command
        nonlocal has_right_arm_command
        nonlocal has_left_arm_command

        if moving_component == RI.TORSO:
            torso_cmd_builder = cmd_builder
            has_torso_command = True

        elif moving_component == RI.RIGHT_ARM:
            right_arm_cmd_builder = cmd_builder
            has_right_arm_command = True

        elif moving_component == RI.LEFT_ARM:
            left_arm_cmd_builder = cmd_builder
            has_left_arm_command = True

        else:
            logging.error(f"Invalid moving_component: '{moving_component}'")
            return False

        return True

    # -------------------------------------------------------------------------
    # 1. Add Cartesian targets
    # -------------------------------------------------------------------------
    if position_targets is not None:
        for i, position_target in enumerate(position_targets):
            moving_component = position_target.get("moving_component")
            reference_link = position_target.get("reference_link")
            target_link = position_target.get("target_link")
            pos = position_target.get("pos")
            rpy = position_target.get("rpy")

            target_minimum_time_PC = position_target.get(
                "minimum_time",
                minimum_time_PC
            )
            target_control_hold_time_PC = position_target.get(
                "control_hold_time",
                control_hold_time_PC
            )
            joint_targets = position_target.get("joint_targets", None)

            if moving_component is None:
                logging.error(f"position_target {i} has no 'moving_component'")
                return False

            if moving_component not in valid_components:
                logging.error(
                    f"position_target {i}['moving_component'] must be "
                    "'torso', 'right_arm', or 'left_arm'"
                )
                return False

            if reference_link is None:
                logging.error(f"position_target {i} has no 'reference_link'")
                return False

            if target_link is None:
                logging.error(f"position_target {i} has no 'target_link'")
                return False

            if pos is None or len(pos) != 3:
                logging.error(f"position_target {i}['pos'] must have 3 elements")
                return False

            if rpy is None or len(rpy) != 3:
                logging.error(f"position_target {i}['rpy'] must have 3 elements")
                return False

            T = make_HT(rpy_to_rotMat(rpy), pos)

            cartesian_cmd_builder = (
                rby.CartesianCommandBuilder()
                .set_minimum_time(target_minimum_time_PC)
                .set_stop_position_tracking_error(stop_position_tracking_error_PC)
                .set_stop_orientation_tracking_error(stop_orientation_tracking_error_PC)
                .add_target(
                    reference_link,
                    target_link,
                    T,
                    linear_velocity_limit_PC,
                    angular_velocity_limit_PC,
                    acceleration_limit_PC,
                )
            )

            if target_control_hold_time_PC is not None:
                cartesian_cmd_builder.set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(target_control_hold_time_PC)
                )

            if joint_targets is not None:
                for joint_name, joint_pos in joint_targets.items():
                    cartesian_cmd_builder.add_joint_position_target(
                        joint_name,
                        float(joint_pos)
                    )
                    logging.info(
                        f"Add Cartesian joint target: "
                        f"'{joint_name}' = {joint_pos}"
                    )

            if not set_component_command(moving_component, cartesian_cmd_builder):
                return False

            logging.info(
                f"Add Position Control target {i}: "
                f"moving_component='{moving_component}', "
                f"'{reference_link}'->'{target_link}', "
                f"pos={pos}, rpy={rpy}, "
                f"minimum_time={target_minimum_time_PC}, "
                f"control_hold_time={target_control_hold_time_PC}"
            )

    # -------------------------------------------------------------------------
    # 2. Add Impedance targets
    # -------------------------------------------------------------------------
    if impedance_targets is not None:
        for i, impedance_target in enumerate(impedance_targets):
            moving_component = impedance_target.get("moving_component")
            reference_link = impedance_target.get("reference_link")
            target_link = impedance_target.get("target_link")
            pos = impedance_target.get("pos")
            rpy = impedance_target.get("rpy")

            translation_weight = impedance_target.get(
                "translation_weight",
                default_translation_weight_IC
            )
            rotation_weight = impedance_target.get(
                "rotation_weight",
                default_rotation_weight_IC
            )
            damping_ratio = impedance_target.get(
                "damping_ratio",
                default_damping_ratio_IC
            )
            target_control_hold_time_IC = impedance_target.get(
                "control_hold_time",
                control_hold_time_IC
            )

            if moving_component is None:
                logging.error(f"impedance_target {i} has no 'moving_component'")
                return False

            if moving_component not in valid_components:
                logging.error(
                    f"impedance_target {i}['moving_component'] must be "
                    "'torso', 'right_arm', or 'left_arm'"
                )
                return False

            if reference_link is None:
                logging.error(f"impedance_target {i} has no 'reference_link'")
                return False

            if target_link is None:
                logging.error(f"impedance_target {i} has no 'target_link'")
                return False

            if pos is None or len(pos) != 3:
                logging.error(f"impedance_target {i}['pos'] must have 3 elements")
                return False

            if rpy is None or len(rpy) != 3:
                logging.error(f"impedance_target {i}['rpy'] must have 3 elements")
                return False

            if translation_weight is None or len(translation_weight) != 3:
                logging.error(
                    f"impedance_target {i}['translation_weight'] must have 3 elements"
                )
                return False

            if rotation_weight is None or len(rotation_weight) != 3:
                logging.error(
                    f"impedance_target {i}['rotation_weight'] must have 3 elements"
                )
                return False

            T = make_HT(rpy_to_rotMat(rpy), pos)

            impedance_cmd_builder = (
                rby.ImpedanceControlCommandBuilder()
                .set_reference_link_name(reference_link)
                .set_link_name(target_link)
                .set_translation_weight(translation_weight)
                .set_rotation_weight(rotation_weight)
                .set_transformation(T)
            )

            # [Important] If 'control_hold_time' is 0 or None, the robot is not moving
            if target_control_hold_time_IC is not None: 
                impedance_cmd_builder.set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(target_control_hold_time_IC)
                )

            if damping_ratio is not None:
                impedance_cmd_builder.set_damping_ratio(float(damping_ratio))

            if not set_component_command(moving_component, impedance_cmd_builder):
                return False

            logging.info(
                f"Add Impedance target {i}: "
                f"moving_component='{moving_component}', "
                f"'{reference_link}'->'{target_link}', "
                f"pos={pos}, rpy={rpy}, "
                f"translation_weight={translation_weight}, "
                f"rotation_weight={rotation_weight}, "
                f"damping_ratio={damping_ratio}, "
                f"control_hold_time={target_control_hold_time_IC}"
            )

    # -------------------------------------------------------------------------
    # 3. Apply component commands
    # -------------------------------------------------------------------------
    if has_torso_command:
        body_cmd_builder.set_torso_command(torso_cmd_builder)

    if has_right_arm_command:
        body_cmd_builder.set_right_arm_command(right_arm_cmd_builder)

    if has_left_arm_command:
        body_cmd_builder.set_left_arm_command(left_arm_cmd_builder)

    if not has_torso_command and not has_right_arm_command and not has_left_arm_command:
        logging.error("No command is applied")
        return False

    # -------------------------------------------------------------------------
    # 4. Build robot command
    # -------------------------------------------------------------------------
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            body_cmd_builder
        )
    )

    # -------------------------------------------------------------------------
    # 5. Send command
    # -------------------------------------------------------------------------
    if get_return:
        rv = robot.send_command(rc).get()

        if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
            logging.error(
                f"Failed to conduct Cartesian (Position + Impedance) Control. "
                f"finish_code = {rv.finish_code}"
            )
            return False

        UTIL.print_end_title("Cartesian (Cartesian + Impedance) Control Ends")
        return True
    else:
        handler = robot.send_command(rc)
        UTIL.print_end_title("Cartesian (Cartesian + Impedance) Control Ends")
        return handler



def optimalCtrl_wholeBody(
    robot,
    position_targets: list[dict] | None = None,
    joint_position_targets: dict | None = None,
    center_of_mass_target: dict | None = None,
    default_translation_weight: float = RI.WEIGHT,
    default_rotation_weight: float = RI.WEIGHT,
    default_joint_weight: float = RI.WEIGHT / 10,
    velocity_limit_scaling: float | None = 0.5,
    error_scaling: float | None = None,
    stop_cost: float = RI.STOP_COST,
    min_delta_cost: float = RI.MIN_DELTA_COST,
    patience: int = RI.PATIENCE,
    get_return: bool = True,
):
    """
    Whole-Body Optimal Control

    This function uses OptimalControlCommandBuilder.
    Multiple Cartesian position targets, Joint position targets, and COM target
    can be solved together using whole-body joints.

    
    When use this function: 
        [hdlr = "function"] form is Necessary when "get_return=False"

    Args:
        position_targets:
            [
                {
                    "reference_link": "base",
                    "target_link": "ee_right",
                    "pos": [x, y, z],
                    "rpy": [roll, pitch, yaw],

                    # optional
                    "translation_weight": RI.WEIGHT,
                    "rotation_weight": RI.WEIGHT,
                },
                {
                    "reference_link": "base",
                    "target_link": "link_torso_5",
                    "pos": [0, 0, 1.0],
                    "rpy": [0, 0, 0],
                    "translation_weight": RI.WEIGHT,
                    "rotation_weight": RI.WEIGHT,
                },
            ]

        joint_position_targets:
            {
                "right_arm_2": {
                    "pos": np.pi / 2,
                    "weight": RI.WEIGHT / 5,
                },
                "left_arm_2": {
                    "pos": -np.pi / 2,
                    "weight": RI.WEIGHT / 5,
                },
            }

        center_of_mass_target:
            {
                "reference_link": "base",
                "pos": [x, y, z],
                "weight": RI.WEIGHT * 5,
            }
            
    velocity_limit_scaling : Limitation percentage of angular velocity (0.5 = 50% of maximum velocity)
    error_scaling          : How sensitively check the error (2 = set error as twice of original error)
    stop_cost              : If cost is smaller than this value, then terminate
    min_delta_cost         : If reduced cost in 1-time step is smaller than this value, we assume it is in stagnant
    patience               : How many times we wait the stagnant state (After over patience, terminate)

    [CMD structure]
        RobotCommandBuilder
        └── ComponentBasedCommandBuilder
            └── OptimalControlCommandBuilder
                ├── Cartesian targets
                ├── Joint position targets
                └── Center of mass target
    """

    UTIL.print_start_title("Whole-Body Optimal Control Starts")

    if (position_targets is None or len(position_targets) == 0) and \
       (joint_position_targets is None or len(joint_position_targets) == 0) and \
       (center_of_mass_target is None):
        logging.error("No optimal control target is given")
        return False

    # 1. Make Optimal Control Command Builder
    optimal_control_cmd_builder = rby.OptimalControlCommandBuilder()

    # 2. Add center of mass target if needed
    if center_of_mass_target is not None:
        com_reference_link = center_of_mass_target.get("reference_link")
        com_pos = center_of_mass_target.get("pos")
        com_weight = center_of_mass_target.get("weight", default_translation_weight)

        if com_reference_link is None:
            logging.error("center_of_mass_target has no 'reference_link'")
            return False

        if com_pos is None or len(com_pos) != 3:
            logging.error("center_of_mass_target['pos'] must have 3 elements")
            return False

        if com_weight is None:
            logging.error("center_of_mass_target has no 'weight'")
            return False

        optimal_control_cmd_builder.set_center_of_mass_target(
            com_reference_link,
            np.asarray(com_pos, dtype=float),
            float(com_weight),
        )

        logging.info(
            f"Add COM target: "
            f"reference_link='{com_reference_link}', "
            f"pos={com_pos}, weight={com_weight}"
        )

    # 3. Add Cartesian Position targets
    if position_targets is not None:
        for i, position_target in enumerate(position_targets):
            reference_link = position_target.get("reference_link")
            target_link = position_target.get("target_link")
            pos = position_target.get("pos")
            rpy = position_target.get("rpy")

            translation_weight = position_target.get(
                "translation_weight",
                default_translation_weight
            )
            rotation_weight = position_target.get(
                "rotation_weight",
                default_rotation_weight
            )

            if reference_link is None:
                logging.error(f"position_target {i} has no 'reference_link'")
                return False

            if target_link is None:
                logging.error(f"position_target {i} has no 'target_link'")
                return False

            if pos is None or len(pos) != 3:
                logging.error(f"position_target {i}['pos'] must have 3 elements")
                return False

            if rpy is None or len(rpy) != 3:
                logging.error(f"position_target {i}['rpy'] must have 3 elements")
                return False

            if translation_weight is None:
                logging.error(f"position_target {i} has no 'translation_weight'")
                return False

            if rotation_weight is None:
                logging.error(f"position_target {i} has no 'rotation_weight'")
                return False

            T = make_HT(rpy_to_rotMat(rpy), pos)

            optimal_control_cmd_builder.add_cartesian_target(
                reference_link,
                target_link,
                T,
                float(translation_weight),
                float(rotation_weight),
            )

            logging.info(
                f"Add Optimal Cartesian Position target {i}: "
                f"'{reference_link}'->'{target_link}', "
                f"pos={pos}, rpy={rpy}, "
                f"translation_weight={translation_weight}, "
                f"rotation_weight={rotation_weight}"
            )

    # 4. Add joint position targets
    if joint_position_targets is not None:
        for joint_name, joint_target in joint_position_targets.items():

            joint_pos = joint_target.get("pos")
            joint_weight = joint_target.get("weight", default_joint_weight)

            if joint_pos is None:
                logging.error(f"joint_position_targets['{joint_name}'] has no 'pos'")
                return False

            if joint_weight is None:
                logging.error(f"joint_position_targets['{joint_name}'] has no 'weight'")
                return False

            optimal_control_cmd_builder.add_joint_position_target(
                joint_name,
                float(joint_pos),
                float(joint_weight),
            )

            logging.info(
                f"Add optimal joint target: "
                f"'{joint_name}' = {joint_pos}, weight={joint_weight}"
            )

    # 5. Set optimal control options
    if velocity_limit_scaling is not None:
        optimal_control_cmd_builder.set_velocity_limit_scaling(
            float(velocity_limit_scaling)
        )

    if error_scaling is not None:
        optimal_control_cmd_builder.set_error_scaling(
            float(error_scaling)
        )

    optimal_control_cmd_builder.set_stop_cost(stop_cost)
    optimal_control_cmd_builder.set_min_delta_cost(min_delta_cost)
    optimal_control_cmd_builder.set_patience(patience)

    logging.info(
        f"Optimal control options: "
        f"velocity_limit_scaling={velocity_limit_scaling}, "
        f"error_scaling={error_scaling}, "
        f"stop_cost={stop_cost}, "
        f"min_delta_cost={min_delta_cost}, "
        f"patience={patience}"
    )

    # 6. Build robot command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            optimal_control_cmd_builder
        )
    )

    # 7. Send command
    if get_return:
        rv = robot.send_command(rc, 10).get()

        if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
            logging.error(
                f"Failed to conduct Whole-Body Optimal Control. "
                f"finish_code = {rv.finish_code}"
            )
            return False

        UTIL.print_end_title("Whole-Body Optimal Control Ends")
        return True
    else:
        handler = robot.send_command(rc, 10)
        UTIL.print_end_title("Whole-Body Optimal Control Ends")
        return handler




def gravityCompensation(
    robot,
    GC_ONOFF = RI.GC_SET_ON,           # ysw.GC_SET_ON or ysw.GC_SET_OFF
    torso: bool = False,
    right_arm: bool = False,
    left_arm: bool = False,
    control_hold_time: float = RI.MINIMUM_TIME,
    get_return: bool = True,
):
    """
    Whole-Body Gravity Compensation Control

    Turn on gravity compensation for selected body components.
    This command does not move the robot to a target pose.
    It only compensates gravity so that the selected components feel lighter.

    When use this function: 
        [hdlr = "function"] form is Necessary when "get_return=False"

    Args:
        GC_ONOFF:
            If ysw.GC_ON, turn on gravity compensation
            If ysw.GC_OFF, turn off gravity compensation
        
        torso:
            If True, turn on gravity compensation for torso.

        right_arm:
            If True, turn on gravity compensation for right arm.

        left_arm:
            If True, turn on gravity compensation for left arm.

        control_hold_time:
            Duration for which gravity compensation is maintained.
        
        get_return:
            If True, wait until the command finishes and return True/False.
            If False, return the command future.

    [CMD structure]
        RobotCommandBuilder
        └── ComponentBasedCommandBuilder
            └── BodyComponentBasedCommandBuilder
                ├── torso_command
                ├── right_arm_command
                └── left_arm_command
    """
    def make_gravityComp_cmd():
        return (
            rby.GravityCompensationCommandBuilder()
            .set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(
                    control_hold_time
                )
            )
            .set_on(GC_ONOFF)
        )
    
    if GC_ONOFF == RI.GC_SET_ON:        GC_state_str = "ON"
    elif GC_ONOFF == RI.GC_SET_OFF:     GC_state_str = "OFF"

    UTIL.print_start_title(f"Gravity Compensation {GC_state_str} Starts")     

    # 1. Check the validity of parameters
    if GC_ONOFF not in [RI.GC_SET_ON, RI.GC_SET_OFF]:
        logging.error("Invalid GC_ONOFF parameter is entered")
        return False
    
    if not torso and not right_arm and not left_arm:
        logging.error("No body component is selected for gravity compensation")
        return False

    # 2. Make Body Component Command Builder
    body_cmd_builder = rby.BodyComponentBasedCommandBuilder()

    # 2. Torso gravity compensation
    if torso:
        body_cmd_builder = body_cmd_builder.set_torso_command(
            make_gravityComp_cmd()
        )
        logging.info(f"'torso' gravity compensation is turned {GC_state_str}")

    # 3. Right arm gravity compensation
    if right_arm:
        body_cmd_builder = body_cmd_builder.set_right_arm_command(
            make_gravityComp_cmd()
        )
        logging.info(f"'right_arm' gravity compensation is turned {GC_state_str}")

    # 4. Left arm gravity compensation
    if left_arm:
        body_cmd_builder = body_cmd_builder.set_left_arm_command(
            make_gravityComp_cmd()
        )
        logging.info(f"'left_arm' gravity compensation is turned {GC_state_str}")

    # 5. Build robot command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            body_cmd_builder
        )
    )

    # 7. Send command
    if get_return:
        rv = robot.send_command(rc).get()

        if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
            logging.error(
                f"Failed to turn {GC_state_str} Gravity Compensation. "
                f"finish_code = {rv.finish_code}"
            )
            return False

        UTIL.print_end_title(f"Gravity Compensation {GC_state_str} Ends")
        return True
    else:
        handler = robot.send_command(rc)
        UTIL.print_end_title(f"Gravity Compensation {GC_state_str} Ends")
        return handler




# def impedanceCtrl_joint_20DoF(
#     robot, 
#     impedance_targets: dict,
#     default_stiffness: float = RI.DEFAULT_STIFFNESS,
#     default_damping_ratio: float = RI.DEFAULT_DAMPING_RATIO,
#     default_torque_limit: float = RI.DEFAULT_TORQUE_LIMIT,
#     default_minimum_time: float = RI.MINIMUM_TIME,
#     default_control_hold_time: float = RI.DEFAULT_CONTROL_HOLD_TIME,     # Important!! 
#     get_return=True
# ):
#     """
#     Joint Impedance Control 

#     Apply joint impedance control to selected joints.

#     When use this function: 
#         [hdlr = "function"] form is Necessary when "get_return=False"    

#     Args:
#         impedance_targets:
#             Dictionary whose keys are component names:
#                 "torso", "right_arm", "left_arm"

#             Example:
#                 {
#                     "torso": {
#                         "pos": [q2, q3, q4, q5, q6, q7],       # Joint angle position
#                         "stiffness": [100.0] * 6,
#                         "damping_ratio": 1.0,
#                         "torque_limit": [10.0] * 6,
#                         "minimum_time": 5.0,
#                         "control_hold_time": 10.0,
#                     }
#                     "right_arm": {
#                         "pos": [q8, q9, q10, q11, q12, q13, q14],
#                         "stiffness": [100.0] * 7,
#                         "damping_ratio": 1.0,
#                         "torque_limit": [10.0] * 7,
#                         "minimum_time": 5.0,
#                         "control_hold_time": 10.0,
#                     }
#                     "left_arm": {
#                         "pos": [q15, q16, q17, q18, q19, q20, q21],
#                         "stiffness": [100.0] * 7,
#                         "damping_ratio": 1.0,
#                         "torque_limit": [10.0] * 7,
#                         "minimum_time": 5.0,
#                         "control_hold_time": 10.0,
#                     }
#                 }

#         default_stiffness:
#             Default joint stiffness.
#             If scalar, it is expanded to the component DoF.

#         default_damping_ratio:
#             Default damping ratio.

#         default_torque_limit:
#             Default joint torque limit.
#             If scalar, it is expanded to the component DoF.

#         default_minimum_time:
#             Default minimum motion time.

#         default_control_hold_time:
#             Default command hold time.

#         get_return:
#             If True, wait until the command finishes and return True/False.
#             If False, return the command handler.

#     Returns:
#         True / False if get_return=True.
#         Command handler if get_return=False.

#     [CMD structure]
#         RobotCommandBuilder
#         └── ComponentBasedCommandBuilder
#             └── BodyComponentBasedCommandBuilder
#                 ├── torso_command
#                 ├── right_arm_command
#                 └── left_arm_command
#     """    
#     UTIL.print_start_title("Joint Impedance Control Starts")

#     # 1. Argument validity check
#     if impedance_targets is None or len(impedance_targets) == 0:
#         logging.error("'impedance_targets' is empty")
#         return False
    
#     component_dof_map = {
#         RI.TORSO: RI.DOF_TORSO,             # 6
#         RI.RIGHT_ARM: RI.DOF_RIGHT_ARM,     # 7
#         RI.LEFT_ARM: RI.DOF_LEFT_ARM,       # 7
#     }
#     valid_components = RI.BODY_COMPONENT_NAME_LIST

#     # 2. Make Body Component Command Builder
#     body_cmd_builder = rby.BodyComponentBasedCommandBuilder()
#     has_command = False

#     # 3. Build component-wise joint impedance commands
#     for component_name, target in impedance_targets.items():

#         if component_name not in valid_components:
#             logging.error(
#                 f"Invalid component_name: '{component_name}'"
#             )
#             return False

#         dof = component_dof_map[component_name]

#         if "pos" not in target:
#             logging.error(
#                 f"'pos' is required for '{component_name}' joint impedance target"
#             )
#             return False
        
#         q_list = list(target["pos"])

#         if len(q_list) != dof:
#             logging.error(
#                 f"'position' length of '{component_name}' must be {dof}, "
#                 f"but got {len(q_list)}"
#             )
#             return False

#         q_list = [float(q) for q in q_list]

#         ### stiffness
#         target_stiffness = target.get("stiffness", default_stiffness)

#         if isinstance(target_stiffness, (int, float)):
#             target_stiffness = [float(target_stiffness)] * dof
#         else:
#             target_stiffness = list(target_stiffness)

#             if len(target_stiffness) != dof:
#                 logging.error(
#                     f"'stiffness' length of '{component_name}' must be {dof}, "
#                     f"but got {len(target_stiffness)}"
#                 )
#                 return False

#             target_stiffness = [float(v) for v in target_stiffness]

#         ### torque_limit
#         target_torque_limit = target.get("torque_limit", default_torque_limit)

#         if isinstance(target_torque_limit, (int, float)):
#             target_torque_limit = [float(target_torque_limit)] * dof
#         else:
#             target_torque_limit = list(target_torque_limit)

#             if len(target_torque_limit) != dof:
#                 logging.error(
#                     f"'torque_limit' length of '{component_name}' must be {dof}, "
#                     f"but got {len(target_torque_limit)}"
#                 )
#                 return False

#             target_torque_limit = [float(v) for v in target_torque_limit]
        
#         target_damping_ratio = float(
#             target.get("damping_ratio", default_damping_ratio)
#         )

#         target_minimum_time = float(
#             target.get("minimum_time", default_minimum_time)
#         )

#         target_control_hold_time = float(
#             target.get("control_hold_time", default_control_hold_time)
#         )

#         joint_impedance_cmd = make_joint_impedance_command(
#             q_list=q_list,
#             stiffness=target_stiffness,
#             damping_ratio=target_damping_ratio,
#             torque_limit=target_torque_limit,
#             minimum_time=target_minimum_time,
#             control_hold_time=target_control_hold_time,
#         )

#         if component_name == RI.TORSO:
#             body_cmd_builder = body_cmd_builder.set_torso_command(
#                 joint_impedance_cmd
#             )

#         elif component_name == RI.RIGHT_ARM:
#             body_cmd_builder = body_cmd_builder.set_right_arm_command(
#                 joint_impedance_cmd
#             )

#         elif component_name == RI.LEFT_ARM:
#             body_cmd_builder = body_cmd_builder.set_left_arm_command(
#                 joint_impedance_cmd
#             )

#         logging.info(f"Joint impedance command is set for '{component_name}'")
#         has_command = True

#     if not has_command:
#         logging.error("No joint impedance command is generated")
#         return False

#     # 4. Build robot command
#     rc = rby.RobotCommandBuilder().set_command(
#         rby.ComponentBasedCommandBuilder().set_body_command(
#             body_cmd_builder
#         )
#     )

#     # 5. Send command
#     if get_return:
#         rv = robot.send_command(rc).get()

#         if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
#             logging.error(
#                 f"Failed to conduct Joint Impedance Control. "
#                 f"finish_code = {rv.finish_code}"
#             )
#             return False

#         UTIL.print_end_title("Joint Impedance Control Ends")
#         return True

#     else:
#         handler = robot.send_command(rc)
#         UTIL.print_end_title("Joint Impedance Control Command Sent")
#         return handler


