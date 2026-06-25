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



def move_position_20dof(robot, posture, minimum_time=5.0, control_hold_time=0.5, get_return=True):
    """
    Move only 20-DoF Body segments (torso, right_arm, left_arm)
    Index: 2 ~ 21
    
    posture:
        {
            "torso": np.array/list,      # 6 dof    [2, 3, 4, 5, 6, 7]
            "right_arm": np.array/list,  # 7 dof    [8, 9, 10, 11, 12, 13, 14]
            "left_arm": np.array/list    # 7 dof    [15, 16, 17, 18, 19, 20, 21]
        }
    """
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

    rc_builder = rby.RobotCommandBuilder().set_command(component_cmd_builder)
    
    if (get_return == True):
        rc = robot.send_command(rc_builder).get()
        
        if rc.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
            logging.error("Failed to conduct [move_position_22dof].")
            return False
        return True
    else:
        rc = robot.send_command(rc_builder)
        return rc




def move_position_22dof(robot, posture, minimum_time=5.0, control_hold_time=0.5, get_return=True):
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
    """
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

    # if not has_body_command and q_head is None:
    #     raise ValueError("No command was provided. At least one posture entry must be not None.")

    rc_builder = rby.RobotCommandBuilder().set_command(component_cmd_builder)
    
    if (get_return == True):
        rc = robot.send_command(rc_builder).get()
        
        if rc.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
            logging.error("Failed to conduct [move_position_22dof].")
            return False
        return True
    else:
        rc = robot.send_command(rc_builder)
        return rc



def move_head(robot, q_head, minimum_time=2.0, control_hold_time=0.5, get_return=True):
    """
    Move only 2-DoF Head segments
    Index: 22 ~ 23
    
    q_head: np.array/list  # 2 dof        [head_0: yaw, head_1: pitch]
    """
    rc_builder = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_head_command(
            rby.JointPositionCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time))
            .set_minimum_time(minimum_time)
            .set_position(q_head)
        )
    )    
    if (get_return == True):
        rc = robot.send_command(rc_builder).get()
        return rc.finish_code == rby.RobotCommandFeedback.FinishCode.Ok
    else:
        rc = robot.send_command(rc_builder)
