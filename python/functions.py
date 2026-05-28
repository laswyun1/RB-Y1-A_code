import rby1_sdk as rby
import time
import numpy as np
from typing import Optional, List
from dynamixel_sdk import COMM_SUCCESS
import cv2
import pyrealsense2 as rs
import logging


ROBOT_ADDRESS = "192.168.30.1:50051"        # For Real Robot
RIGHT_END     = 0       # Right End Effector idx
LEFT_END      = 1       # Left End Effector idx


#----- 22-DoF Posture -----#
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


#----- 2-DoF Head Posture -----#
HEAD_POS_DEMO = [0.0, np.deg2rad(30.0)] 
HEAD_ZERO_POS = [0.0, 0.0]


#----- Setting Parameters -----#
EE_OFFSET_X_DEMO = 0.105    # End-effector offset

TURN_RIGHT_90   = -83.5
TURN_LEFT_90    = 83.5


#----- Gripper -----# 
PROTOCOL_VERSION = 2.0
BAUDRATE = 2000000
DEVICE_NAME_GRIPPER = "/dev/rby1_gripper"

ADDR_OPERATING_MODE     = 11                # 0: Current Control / 1: Velocity Control / 3: Position Control(default)
ADDR_CURRENT_LIMIT      = 38                # 0 ~ 1193
ADDR_TORQUE_ENABLE      = 64                # 0: Turn OFF(default) / 1: Turn ON
ADDR_LED                = 65                # 0: Turn OFF(default) / 1: Turn ON
ADDR_GOAL_CURRENT       = 102
ADDR_GOAL_POSITION      = 116   
ADDR_PRESENT_CURRENT    = 126               # Unit: 2.69mA
ADDR_PRESENT_POSITION   = 132               # Unit: 1-pulse(or Tick)

CURRENT_CONTROL_MODE    = 0
POSITION_CONTROL_MODE   = 3
TORQUE_DISABLE          = 0
TORQUE_ENABLE           = 1

DXL_MIN_POS_VALUE       = 0
DXL_MAX_POS_VALUE       = 4095

RESOLUTION_TICKS_PER_REV    = 4096.0
TWO_PI                      = 2.0 * np.pi
RAD2DEG                     = 180.0 / np.pi
CURRENT_LIMIT_MAX_A         = 3.2               # Unit: A
CURRENT_LIMIT_MAX_RAW       = 1193              # Unit: 2.69mA

R_DXL_ID                    = 0
L_DXL_ID                    = 1

is_init_finish              = False

DEMO_GRIP_CURRENT_R         = 0.8
DEMO_GRIP_CURRENT_L         = 0.6
DEMO_LAY_CURRENT_R          = -0.04
DEMO_LAY_CURRENT_L          = -0.03
DEMO_ZEROING_CURRENT_L      = 0.03
DEMO_LAY_CURRENT_END        = 0.0


#----- Functions -------------------------# 
def move_head(robot, q_head, minimum_time=2.0, get_return=True):
    """
        q_head: np.array/list  # 2 dof, e.g. [head_0: yaw, head_1: pitch]
    """
    rc_builder = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_head_command(
            rby.JointPositionCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(2.0))
            .set_minimum_time(minimum_time)
            .set_position(q_head)
        )
    )    
    if (get_return == True):
        rc = robot.send_command(rc_builder).get()
        return rc.finish_code == rby.RobotCommandFeedback.FinishCode.Ok
    else:
        rc = robot.send_command(rc_builder)
    
    
    
def move_neck_example(robot):
    # [head_0, head_1]
    q_head_center = [0.0, 0.0]
    q_head_left = [np.deg2rad(20.0), 0.0]
    q_head_right = [np.deg2rad(-20.0), 0.0]
    q_head_up = [0.0, np.deg2rad(10.0)]
    q_head_down = [0.0, np.deg2rad(-10.0)]
    

    move_head(robot, q_head_center)
    time.sleep(1)
    move_head(robot, q_head_left)
    time.sleep(1)
    move_head(robot, q_head_right)
    time.sleep(1)
    move_head(robot, q_head_up)
    time.sleep(1)
    move_head(robot, q_head_down)
    time.sleep(1)    



def make_joint_position_command(q_list, minimum_time=5.0, control_hold_time=0.5):
    return (rby.JointPositionCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time))
            .set_minimum_time(minimum_time)
            .set_position(q_list)
    )


def move_position_20dof(robot, posture, minimum_time=5.0, control_hold_time=0.5, get_return=True):
    """
    posture:
        {
            "torso": np.array/list,      # 6 dof
            "right_arm": np.array/list,  # 7 dof
            "left_arm": np.array/list    # 7 dof
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
    posture:
        {
            "torso": np.array/list,      # 6 dof
            "right_arm": np.array/list,  # 7 dof
            "left_arm": np.array/list    # 7 dof
            "head": np.array/list        # 2 dof (0: yaw, 1: pitch)
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



def build_mobility_cmd(vx, vy=0.0, yaw_rate=0.0, hold_time=0.3, acc=0.2):
    return rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_mobility_command(
            rby.SE2VelocityCommandBuilder()
            .set_velocity(np.array([vx, vy]), yaw_rate)
            .set_acceleration_limit(np.array([acc, acc]), acc)
            .set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(hold_time)
            )
        )
    )
    
    
    
def move_forward_distance_smooth(
    robot,
    distance_m,
    max_speed=0.3,       # m/s
    accel=0.1,           # m/s^2, 작을수록 천천히 가속
    tolerance=0.01,
    dt=0.02              # 50 Hz
):
    robot.reset_odometry(
        angle=0.0,
        position=np.array([0.0, 0.0])
    )
    # For skipping Move function due to malfunction of reset odometry
    for i in range(20):
        odom = robot.get_state().odometry
        x = odom[0, 2]
        y = odom[1, 2]
        if np.sqrt(x*x + y*y) < 0.02:
            break;
        time.sleep(0.2)
    
    direction = 1.0 if distance_m >= 0 else -1.0
    target = abs(distance_m)
    current_speed = 0.0

    stream = robot.create_command_stream(priority=1)

    try:
        while True:
            odom = robot.get_state().odometry
            x = odom[0, 2]
            y = odom[1, 2]

            moved = np.sqrt(x*x + y*y)
            remain = target - moved

            if remain <= tolerance:
                break

            # 남은 거리에서 안전하게 멈출 수 있는 속도
            # v^2 = 2 a x
            brake_speed = np.sqrt(max(0.0, 2.0 * accel * remain))

            # 목표 속도: 최대속도와 제동 가능 속도 중 작은 값
            target_speed = min(max_speed, brake_speed)
            
            print(f"moved={moved:.3f}, speed={current_speed:.3f}")

            # 한 루프마다 속도 변화량 제한
            max_dv = accel * dt

            if target_speed > current_speed:
                current_speed = min(current_speed + max_dv, target_speed)
            else:
                current_speed = max(current_speed - max_dv, target_speed)

            vx = direction * current_speed

            cmd = build_mobility_cmd(
                vx,
                0.0,
                0.0,
                hold_time=0.3,
                acc=accel
            )

            # 핵심: .get()으로 매번 기다리지 않고 stream으로 연속 전송
            stream.send_command(cmd, timeout_ms=500)

            time.sleep(dt)

        # 마지막 감속 정지
        # (1) 일반적인 선형 감속
        # for _ in range(20):
        #     current_speed = max(0.0, current_speed - accel * dt)
        #     vx = direction * current_speed

        #     cmd = build_mobility_cmd(
        #         vx,
        #         0.0,
        #         0.0,
        #         hold_time=0.3,
        #         acc=accel
        #     )
        #     stream.send_command(cmd, timeout_ms=500)
        #     time.sleep(dt)
        
        # (2) 더 부드러운 ease-out
        stop_duration = 2.0  # 멈추는 데 걸리는 시간, 길수록 부드러움
        steps = int(stop_duration / dt)

        start_speed = current_speed

        for i in range(steps):
            t = (i + 1) / steps  # 0 → 1

            # smoothstep: 처음/끝 변화가 부드러움
            s = t * t * (3 - 2 * t)

            current_speed = start_speed * (1.0 - s)
            vx = direction * current_speed

            cmd = build_mobility_cmd(
                vx,
                0.0,
                0.0,
                hold_time=0.3,
                acc=accel
            )
            stream.send_command(cmd, timeout_ms=1000)
            time.sleep(dt)

    finally:
        # 완전 정지 명령 여러 번 보내기
        for _ in range(5):
            stream.send_command(
                build_mobility_cmd(0.0, 0.0, 0.0, hold_time=0.5, acc=accel),
                timeout_ms=1000
            )
            time.sleep(0.05)

        stream.cancel()
        
        
        
def get_yaw(odom):
    return np.arctan2(odom[1, 0], odom[0, 0])



def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi



def rotate_inplace_smooth(
    robot,
    target_deg=90,
    max_yaw_rate=0.3,
    min_yaw_rate=0.05,
    accel=0.15,
    tolerance_deg=1.0,
    dt=0.02
):
    robot.reset_odometry(angle=0.0, position=np.array([0.0, 0.0]))

    target_rad = np.deg2rad(target_deg)
    tolerance = np.deg2rad(tolerance_deg)

    current_rate = 0.0
    stream = robot.create_command_stream(priority=1)

    try:
        while True:
            odom = robot.get_state().odometry
            theta = get_yaw(odom)

            error = normalize_angle(target_rad - theta)

            print(
                f"theta_deg={np.rad2deg(theta):.2f}, "
                f"error_deg={np.rad2deg(error):.2f}, "
                f"rate={current_rate:.3f}"
            )

            if abs(error) < tolerance:
                break

            direction = np.sign(error)

            brake_rate = np.sqrt(max(0.0, 2.0 * accel * abs(error)))
            target_rate = min(max_yaw_rate, brake_rate)

            # 너무 작은 회전속도는 실제 로봇에서 안 먹을 수 있어서 최소값 부여
            target_rate = max(min_yaw_rate, target_rate)

            max_dr = accel * dt
            rate_mag = abs(current_rate)

            if target_rate > rate_mag:
                rate_mag = min(rate_mag + max_dr, target_rate)
            else:
                rate_mag = max(rate_mag - max_dr, target_rate)

            current_rate = direction * rate_mag
            yaw_rate = current_rate

            cmd = build_mobility_cmd(
                0.0,
                0.0,
                yaw_rate,
                hold_time=0.3,
                acc=accel
            )

            stream.send_command(cmd, timeout_ms=500)
            time.sleep(dt)

    finally:
        # 완전 정지
        for _ in range(5):
            stream.send_command(
                build_mobility_cmd(0.0, 0.0, 0.0, hold_time=0.5),
                timeout_ms=1000
            )
            time.sleep(0.05)

        stream.cancel()
        
        
        
# Moving arm (0: Right)
def move_endeffector(robot, direction_ee, rel_delta_pos):
    dyn_model = robot.get_dynamics()
    dyn_state = dyn_model.make_state(["base", "link_torso_5", "ee_right", "ee_left"], robot.model().robot_joint_names)
    BASE_INDEX, LINK_TORSO_5_INDEX, EE_RIGHT_INDEX, EE_LEFT_INDEX = 0, 1, 2, 3

    dyn_state.set_q(robot.get_state().position)         # Get current gc
    dyn_model.compute_forward_kinematics(dyn_state)     # FK
    
    T_delta = np.eye(4)
    T_delta[0:3, 3] = rel_delta_pos
    
    if (direction_ee == RIGHT_END):
        T_torso2ee = dyn_model.compute_transformation(
            dyn_state, 
            LINK_TORSO_5_INDEX, 
            EE_RIGHT_INDEX
        )
        ee_builder = (
            rby.CartesianCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(100))
            .set_minimum_time(3)
            .add_target("link_torso_5", "ee_right", T_torso2ee @ T_delta, 1.0, 1.0, 1.0)
            .add_joint_position_target(robot.model().robot_joint_names[robot.model().right_arm_idx[2]], 0)
        )
        return rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder()
            .set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_right_arm_command(ee_builder)
            )
        )
    elif (direction_ee == LEFT_END):
        T_torso2ee = dyn_model.compute_transformation(
            dyn_state, 
            LINK_TORSO_5_INDEX, 
            EE_LEFT_INDEX
        )
        ee_builder = (
            rby.CartesianCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(100))
            .set_minimum_time(3)
            .add_target("link_torso_5", "ee_left", T_torso2ee @ T_delta, 1.0, 1.0, 1.0)
            .add_joint_position_target(robot.model().robot_joint_names[robot.model().left_arm_idx[2]], 0)
        )
        return rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder()
            .set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_left_arm_command(ee_builder)
            )
        )

    return None



# rel_delta_pos: endeffector 기준 pos, R_torso_target: torso5 기준 ori
# def move_to_firstblock(robot, direction_ee, rel_delta_pos, RPY_torso_target):
#     dyn_model = robot.get_dynamics()
#     dyn_state = dyn_model.make_state(["base", "link_torso_5", "ee_right", "ee_left"], robot.model().robot_joint_names)
#     BASE_INDEX, LINK_TORSO_5_INDEX, EE_RIGHT_INDEX, EE_LEFT_INDEX = 0, 1, 2, 3
    
#     dyn_state.set_q(robot.get_state().position)         # Get current gc
#     dyn_model.compute_forward_kinematics(dyn_state)     # FK
    
#     T_torso2ee_now = dyn_model.compute_transformation(
#         dyn_state,
#         LINK_TORSO_5_INDEX,
#         EE_LEFT_INDEX
#     )

#     R_torso2ee_now = T_torso2ee_now[0:3, 0:3]
#     p_torso2ee_now = T_torso2ee_now[0:3, 3]
    
#     p_delta_ee = np.array(rel_delta_pos)          # ee_left 기준 이동량
#     p_delta_torso = R_torso2ee_now @ p_delta_ee   # torso5 기준으로 변환

#     p_target = p_torso2ee_now + p_delta_torso
#     R_target = rpy_to_rotmat(RPY_torso_target[0], RPY_torso_target[1], RPY_torso_target[2])
    
#     T_target = np.eye(4)
#     T_target[0:3, 3] = p_target
#     T_target[0:3, 0:3] = np.eye(3)
    
    
#     ee_builder = (
#         rby.CartesianCommandBuilder()
#         .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1))
#         .set_minimum_time(3)
#         .add_target("link_torso_5", "ee_left", T_target, 1.0, 1.0, 1.0)
#         .add_joint_position_target(robot.model().robot_joint_names[robot.model().left_arm_idx[2]], 0)
#     )
    
#     rc_builder = rby.RobotCommandBuilder().set_command(
#         rby.ComponentBasedCommandBuilder()
#         .set_body_command(
#             rby.BodyComponentBasedCommandBuilder()
#             .set_left_arm_command(ee_builder)
#         )
#     )
    
#     rc = robot.send_command(rc_builder).get()

#     print("finish_code:", rc.finish_code)
#     print("feedback:", rc)

#     return rc.finish_code
    
    
    
def rpy_to_rotmat(roll, pitch, yaw):
    Rx = np.array([
        [1,       0,            0      ],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])

    Ry = np.array([
        [np.cos(pitch),  0, np.sin(pitch)],
        [0,              1,       0      ],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,                  0,     1]
    ])

    return Rz @ Ry @ Rx   # xyz convention



# rel_delta_pos: end-effector 기준 position delta
def move_arm_pos(robot, direction_ee, rel_delta_pos):
    dyn_model = robot.get_dynamics()
    model = robot.model()

    dyn_state = dyn_model.make_state(
        ["link_torso_5", "ee_right", "ee_left"],
        model.robot_joint_names
    )

    LINK_TORSO_5_INDEX, EE_RIGHT_INDEX, EE_LEFT_INDEX = 0, 1, 2

    dyn_state.set_q(robot.get_state().position)
    dyn_model.compute_forward_kinematics(dyn_state)

    # left arm 기준
    if (direction_ee == RIGHT_END):
        T_torso2ee = dyn_model.compute_transformation(
            dyn_state,
            LINK_TORSO_5_INDEX,
            EE_RIGHT_INDEX
        )
    elif (direction_ee == LEFT_END):
        T_torso2ee = dyn_model.compute_transformation(
            dyn_state,
            LINK_TORSO_5_INDEX,
            EE_LEFT_INDEX
        )

    rotmat_torso2ee = T_torso2ee[0:3, 0:3]
    pos_torso2ee = T_torso2ee[0:3, 3]

    # ee기준 delta position을 torso5 기준으로 변환
    pos_delta_ee = np.asarray(rel_delta_pos, dtype=float)
    pos_delta_torso = rotmat_torso2ee @ pos_delta_ee

    pos_target = pos_torso2ee + pos_delta_torso

    # 자세는 현재 자세 유지
    T_target = np.eye(4)
    T_target[0:3, 3] = pos_target
    T_target[0:3, 0:3] = rotmat_torso2ee

    if (direction_ee == RIGHT_END):
        ee_builder = (
            rby.CartesianCommandBuilder()
            .set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(3)
            )
            .set_minimum_time(3.0)
            .add_target("link_torso_5", "ee_right", T_target, 1.0, 1.0, 1.0)
            # .add_joint_position_target(robot.model().robot_joint_names[robot.model().right_arm_idx[2]], 0)
        )
        rc_builder = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder()
            .set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_right_arm_command(ee_builder)
            )
        )
    elif (direction_ee == LEFT_END):
        ee_builder = (
            rby.CartesianCommandBuilder()
            .set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(3)
            )
            .set_minimum_time(3.0)
            .add_target("link_torso_5", "ee_left", T_target, 1.0, 1.0, 1.0)
            # .add_joint_position_target(robot.model().robot_joint_names[robot.model().left_arm_idx[2]], 0)
        )
        rc_builder = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder()
            .set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_left_arm_command(ee_builder)
            )
        )

    rc = robot.send_command(rc_builder).get()
    return rc.finish_code == rby.RobotCommandFeedback.FinishCode.Ok



# rel_pos: end-effector frame based delta position, rel_rpy: end-effector frame based delta position, 
def move_arm_6D_relpos_relori(robot, direction_ee, rel_pos, rel_rpy):
    dyn_model = robot.get_dynamics()
    model = robot.model()

    dyn_state = dyn_model.make_state(
        ["link_torso_5", "ee_right", "ee_left"],
        model.robot_joint_names
    )

    LINK_TORSO_5_INDEX, EE_RIGHT_INDEX, EE_LEFT_INDEX = 0, 1, 2

    dyn_state.set_q(robot.get_state().position)
    dyn_model.compute_forward_kinematics(dyn_state)

    # left arm 기준
    if (direction_ee == RIGHT_END):
        T_torso2ee = dyn_model.compute_transformation(
            dyn_state,
            LINK_TORSO_5_INDEX,
            EE_RIGHT_INDEX
        )
    elif (direction_ee == LEFT_END):
        T_torso2ee = dyn_model.compute_transformation(
            dyn_state,
            LINK_TORSO_5_INDEX,
            EE_LEFT_INDEX
        )

    rotmat_torso2ee = T_torso2ee[0:3, 0:3]
    pos_torso2ee = T_torso2ee[0:3, 3]

    # ee기준 delta position을 torso5 기준으로 변환
    pos_delta_ee = np.asarray(rel_pos, dtype=float)
    pos_delta_torso = rotmat_torso2ee @ pos_delta_ee
    pos_target = pos_torso2ee + pos_delta_torso
    
    # ee기준 delta orientation을 torso5 기준으로 변환
    rotmat_delta_ee = rpy_to_rotmat(rel_rpy[0], rel_rpy[1], rel_rpy[2])
    rotmat_delta_torso = rotmat_torso2ee @ rotmat_delta_ee
    rotmat_target = rotmat_delta_torso
    
    T_target = np.eye(4)
    T_target[0:3, 3] = pos_target
    T_target[0:3, 0:3] = rotmat_target

    if (direction_ee == RIGHT_END):
        ee_builder = (
            rby.CartesianCommandBuilder()
            .set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(0.1)
            )
            .set_minimum_time(3.0)
            .add_target("link_torso_5", "ee_right", T_target, 1.0, 1.0, 1.0)
            # .add_joint_position_target(robot.model().robot_joint_names[robot.model().right_arm_idx[2]], 0)
        )
        rc_builder = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder()
            .set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_right_arm_command(ee_builder)
            )
        )
    elif (direction_ee == LEFT_END):
        ee_builder = (
            rby.CartesianCommandBuilder()
            .set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(0.1)
            )
            .set_minimum_time(3.0)
            .add_target("link_torso_5", "ee_left", T_target, 1.0, 1.0, 1.0)
            # .add_joint_position_target(robot.model().robot_joint_names[robot.model().left_arm_idx[2]], 0)
        )
        rc_builder = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder()
            .set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_left_arm_command(ee_builder)
            )
        )

    rc = robot.send_command(rc_builder).get()
    return rc.finish_code == rby.RobotCommandFeedback.FinishCode.Ok



# rel_pos: end-effector frame based delta position, rpy_global: torso5 based orientation, 
def move_arm_6D_relpos_globalori(robot, direction_ee, rel_pos, global_rpy):
    dyn_model = robot.get_dynamics()
    model = robot.model()

    dyn_state = dyn_model.make_state(
        ["link_torso_5", "ee_right", "ee_left"],
        model.robot_joint_names
    )

    LINK_TORSO_5_INDEX, EE_RIGHT_INDEX, EE_LEFT_INDEX = 0, 1, 2

    dyn_state.set_q(robot.get_state().position)
    dyn_model.compute_forward_kinematics(dyn_state)

    # left arm 기준
    if (direction_ee == RIGHT_END):
        T_torso2ee = dyn_model.compute_transformation(
            dyn_state,
            LINK_TORSO_5_INDEX,
            EE_RIGHT_INDEX
        )
    elif (direction_ee == LEFT_END):
        T_torso2ee = dyn_model.compute_transformation(
            dyn_state,
            LINK_TORSO_5_INDEX,
            EE_LEFT_INDEX
        )

    rotmat_torso2ee = T_torso2ee[0:3, 0:3]
    pos_torso2ee = T_torso2ee[0:3, 3]

    # ee기준 delta position을 torso5 기준으로 변환
    pos_delta_ee = np.asarray(rel_pos, dtype=float)
    pos_delta_torso = rotmat_torso2ee @ pos_delta_ee
    pos_target = pos_torso2ee + pos_delta_torso
    
    # torso5 기준 desired orientation
    rotmat_target = rpy_to_rotmat(global_rpy[0], global_rpy[1], global_rpy[2])
    rotmat_target = rotmat_torso2ee
    
    T_target = np.eye(4)
    T_target[0:3, 3] = pos_target
    T_target[0:3, 0:3] = rotmat_target
    print(f"T_target\n: {T_target}")

    if (direction_ee == RIGHT_END):
        ee_builder = (
            rby.CartesianCommandBuilder()
            .set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(0.1)
            )
            .set_minimum_time(3.0)
            .add_target("link_torso_5", "ee_right", T_target, 1.0, 1.0, 1.0)
            # .add_joint_position_target(robot.model().robot_joint_names[robot.model().right_arm_idx[2]], 0)
        )
        rc_builder = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder()
            .set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_right_arm_command(ee_builder)
            )
        )
    elif (direction_ee == LEFT_END):
        ee_builder = (
            rby.CartesianCommandBuilder()
            .set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(0.1)
            )
            .set_minimum_time(3.0)
            .add_target("link_torso_5", "ee_left", T_target, 1.0, 1.0, 1.0)
            # .add_joint_position_target(robot.model().robot_joint_names[robot.model().left_arm_idx[2]], 0)
        )
        rc_builder = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder()
            .set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_left_arm_command(ee_builder)
            )
        )

    rc = robot.send_command(rc_builder).get()
    return rc.finish_code == rby.RobotCommandFeedback.FinishCode.Ok



# rel_pos: end-effector frame based delta position
def move_arm_relpos(robot, direction_ee, rel_pos):
    dyn_model = robot.get_dynamics()
    model = robot.model()

    dyn_state = dyn_model.make_state(
        ["link_torso_5", "ee_right", "ee_left"],
        model.robot_joint_names
    )

    LINK_TORSO_5_INDEX, EE_RIGHT_INDEX, EE_LEFT_INDEX = 0, 1, 2

    dyn_state.set_q(robot.get_state().position)
    dyn_model.compute_forward_kinematics(dyn_state)

    # left arm 기준
    if (direction_ee == RIGHT_END):
        T_torso2ee = dyn_model.compute_transformation(
            dyn_state,
            LINK_TORSO_5_INDEX,
            EE_RIGHT_INDEX
        )
    elif (direction_ee == LEFT_END):
        T_torso2ee = dyn_model.compute_transformation(
            dyn_state,
            LINK_TORSO_5_INDEX,
            EE_LEFT_INDEX
        )

    pos_torso2ee = T_torso2ee[0:3, 3]
    rotmat_torso2ee = T_torso2ee[0:3, 0:3]

    # ee기준 delta position을 torso5 기준으로 변환
    pos_delta_ee = np.asarray(rel_pos, dtype=float)
    pos_delta_torso = rotmat_torso2ee @ pos_delta_ee
    pos_target = pos_torso2ee + pos_delta_torso
    
    T_target = np.eye(4)
    T_target[0:3, 3] = pos_target
    T_target[0:3, 0:3] = rotmat_torso2ee
    print(f"T_target\n: {T_target}")

    if (direction_ee == RIGHT_END):
        ee_builder = (
            rby.CartesianCommandBuilder()
            .set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(0.1)
            )
            .set_minimum_time(3.0)
            .add_target("link_torso_5", "ee_right", T_target, 1.0, 1.0, 1.0)
            # .add_joint_position_target(robot.model().robot_joint_names[robot.model().right_arm_idx[2]], 0)
        )
        rc_builder = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder()
            .set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_right_arm_command(ee_builder)
            )
        )
    elif (direction_ee == LEFT_END):
        ee_builder = (
            rby.CartesianCommandBuilder()
            .set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(0.1)
            )
            .set_minimum_time(3.0)
            .add_target("link_torso_5", "ee_left", T_target, 1.0, 1.0, 1.0)
            # .add_joint_position_target(robot.model().robot_joint_names[robot.model().left_arm_idx[2]], 0)
        )
        rc_builder = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder()
            .set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_left_arm_command(ee_builder)
            )
        )

    rc = robot.send_command(rc_builder).get()
    return rc.finish_code == rby.RobotCommandFeedback.FinishCode.Ok



# global_pos: torso5 frame based delta position
def move_arm_globalpos(robot, direction_ee, global_pos):
    dyn_model = robot.get_dynamics()
    model = robot.model()

    dyn_state = dyn_model.make_state(
        ["link_torso_5", "ee_right", "ee_left"],
        model.robot_joint_names
    )

    LINK_TORSO_5_INDEX, EE_RIGHT_INDEX, EE_LEFT_INDEX = 0, 1, 2

    dyn_state.set_q(robot.get_state().position)
    dyn_model.compute_forward_kinematics(dyn_state)

    # left arm 기준
    if (direction_ee == RIGHT_END):
        T_torso2ee = dyn_model.compute_transformation(
            dyn_state,
            LINK_TORSO_5_INDEX,
            EE_RIGHT_INDEX
        )
    elif (direction_ee == LEFT_END):
        T_torso2ee = dyn_model.compute_transformation(
            dyn_state,
            LINK_TORSO_5_INDEX,
            EE_LEFT_INDEX
        )

    pos_torso2ee = T_torso2ee[0:3, 3]
    rotmat_torso2ee = T_torso2ee[0:3, 0:3]

    # torso5기준 delta position을 Add
    pos_delta_torso = np.asarray(global_pos, dtype=float)
    pos_target = pos_torso2ee + pos_delta_torso
    
    T_target = np.eye(4)
    T_target[0:3, 3] = pos_target
    T_target[0:3, 0:3] = rotmat_torso2ee
    print(f"T_target\n: {T_target}")

    if (direction_ee == RIGHT_END):
        ee_builder = (
            rby.CartesianCommandBuilder()
            .set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(0.1)
            )
            .set_minimum_time(3.0)
            .add_target("link_torso_5", "ee_right", T_target, 1.0, 1.0, 1.0)
            # .add_joint_position_target(robot.model().robot_joint_names[robot.model().right_arm_idx[2]], 0)
        )
        rc_builder = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder()
            .set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_right_arm_command(ee_builder)
            )
        )
    elif (direction_ee == LEFT_END):
        ee_builder = (
            rby.CartesianCommandBuilder()
            .set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(0.1)
            )
            .set_minimum_time(3.0)
            .add_target("link_torso_5", "ee_left", T_target, 1.0, 1.0, 1.0)
            # .add_joint_position_target(robot.model().robot_joint_names[robot.model().left_arm_idx[2]], 0)
        )
        rc_builder = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder()
            .set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_left_arm_command(ee_builder)
            )
        )

    rc = robot.send_command(rc_builder).get()
    return rc.finish_code == rby.RobotCommandFeedback.FinishCode.Ok



# ori: torso5 frame based orientation
def move_arm_globalori(robot, direction_ee, global_ori):
    dyn_model = robot.get_dynamics()
    model = robot.model()

    dyn_state = dyn_model.make_state(
        ["link_torso_5", "ee_right", "ee_left"],
        model.robot_joint_names
    )

    LINK_TORSO_5_INDEX, EE_RIGHT_INDEX, EE_LEFT_INDEX = 0, 1, 2

    dyn_state.set_q(robot.get_state().position)
    dyn_model.compute_forward_kinematics(dyn_state)

    # left arm 기준
    if (direction_ee == RIGHT_END):
        T_torso2ee = dyn_model.compute_transformation(
            dyn_state,
            LINK_TORSO_5_INDEX,
            EE_RIGHT_INDEX
        )
    elif (direction_ee == LEFT_END):
        T_torso2ee = dyn_model.compute_transformation(
            dyn_state,
            LINK_TORSO_5_INDEX,
            EE_LEFT_INDEX
        )

    pos_torso2ee = T_torso2ee[0:3, 3]
    rotmat_torso2ee = T_torso2ee[0:3, 0:3]

    # ee기준 delta position을 torso5 기준으로 변환
    pos_target = pos_torso2ee
    
    rotmat_target = rpy_to_rotmat(global_ori[0], global_ori[1], global_ori[2])
    
    T_target = np.eye(4)
    T_target[0:3, 3] = pos_target
    T_target[0:3, 0:3] = rotmat_target
    print(f"T_target\n: {T_target}")

    if (direction_ee == RIGHT_END):
        ee_builder = (
            rby.CartesianCommandBuilder()
            .set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(0.1)
            )
            .set_minimum_time(3.0)
            .add_target("link_torso_5", "ee_right", T_target, 1.0, 1.0, 1.0)
            # .add_joint_position_target(robot.model().robot_joint_names[robot.model().right_arm_idx[2]], 0)
        )
        rc_builder = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder()
            .set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_right_arm_command(ee_builder)
            )
        )
    elif (direction_ee == LEFT_END):
        ee_builder = (
            rby.CartesianCommandBuilder()
            .set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(0.1)
            )
            .set_minimum_time(3.0)
            .add_target("link_torso_5", "ee_left", T_target, 1.0, 1.0, 1.0)
            # .add_joint_position_target(robot.model().robot_joint_names[robot.model().left_arm_idx[2]], 0)
        )
        rc_builder = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder()
            .set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_left_arm_command(ee_builder)
            )
        )

    rc = robot.send_command(rc_builder).get()
    return rc.finish_code == rby.RobotCommandFeedback.FinishCode.Ok



# global_pos: torso5 frame based delta position
def move_arm_globalpos_both(robot, global_pos_R, global_pos_L):
    dyn_model = robot.get_dynamics()
    model = robot.model()

    dyn_state = dyn_model.make_state(
        ["link_torso_5", "ee_right", "ee_left"],
        model.robot_joint_names
    )

    LINK_TORSO_5_INDEX, EE_RIGHT_INDEX, EE_LEFT_INDEX = 0, 1, 2

    dyn_state.set_q(robot.get_state().position)
    dyn_model.compute_forward_kinematics(dyn_state)

    T_torso2ee_R = dyn_model.compute_transformation(
        dyn_state,
        LINK_TORSO_5_INDEX,
        EE_RIGHT_INDEX
    )
    T_torso2ee_L = dyn_model.compute_transformation(
        dyn_state,
        LINK_TORSO_5_INDEX,
        EE_LEFT_INDEX
    )

    pos_torso2ee_R = T_torso2ee_R[0:3, 3]
    rotmat_torso2ee_R = T_torso2ee_R[0:3, 0:3]
    
    pos_torso2ee_L = T_torso2ee_L[0:3, 3]
    rotmat_torso2ee_L = T_torso2ee_L[0:3, 0:3]
    
    # torso5기준 delta position을 Add
    pos_delta_torso_R = np.asarray(global_pos_R, dtype=float)
    pos_target_R = pos_torso2ee_R + pos_delta_torso_R
    pos_delta_torso_L= np.asarray(global_pos_L, dtype=float)
    pos_target_L = pos_torso2ee_L + pos_delta_torso_L
    
    T_target_R = np.eye(4)
    T_target_R[0:3, 3] = pos_target_R
    T_target_R[0:3, 0:3] = rotmat_torso2ee_R
    T_target_L = np.eye(4)
    T_target_L[0:3, 3] = pos_target_L
    T_target_L[0:3, 0:3] = rotmat_torso2ee_L
    
    print(f"T_target_R\n: {T_target_R}")
    print(f"T_target_L\n: {T_target_L}")

    ee_builder_R = (
        rby.CartesianCommandBuilder()
        .set_command_header(
            rby.CommandHeaderBuilder().set_control_hold_time(1)
        )
        .set_minimum_time(3.0)
        .add_target("link_torso_5", "ee_right", T_target_R, 1.0, 1.0, 1.0)
        # .add_joint_position_target(robot.model().robot_joint_names[robot.model().right_arm_idx[2]], 0)
    )
    ee_builder_L = (
        rby.CartesianCommandBuilder()
        .set_command_header(
            rby.CommandHeaderBuilder().set_control_hold_time(1)
        )
        .set_minimum_time(3.0)
        .add_target("link_torso_5", "ee_left", T_target_L, 1.0, 1.0, 1.0)
        # .add_joint_position_target(robot.model().robot_joint_names[robot.model().left_arm_idx[2]], 0)
    )
    
    rc_builder = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder()
        .set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            .set_right_arm_command(ee_builder_R)
            .set_left_arm_command(ee_builder_L)
        )
    )

    rc = robot.send_command(rc_builder).get()
    return rc.finish_code == rby.RobotCommandFeedback.FinishCode.Ok



def Get_T_left_ee_from_torso5(robot):
    dyn_model = robot.get_dynamics()
    model = robot.model()

    dyn_state = dyn_model.make_state(
        ["link_torso_5", "ee_left"],
        model.robot_joint_names
    )

    LINK_TORSO_5_INDEX, EE_LEFT_INDEX = 0, 1

    dyn_state.set_q(robot.get_state().position)
    dyn_model.compute_forward_kinematics(dyn_state)

    T_torso2ee_L = dyn_model.compute_transformation(
        dyn_state,
        LINK_TORSO_5_INDEX,
        EE_LEFT_INDEX
    )
    
    return T_torso2ee_L






#----- Functions for Gripper ---------------------------#
def DXL_write_torque_mode(dxl_id: int, torque_mode: int, port_handler, packet_handler) -> None:
    dxl_comm_res, dxl_err = packet_handler.write1ByteTxRx(
        port_handler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        int(torque_mode)
    )
    if dxl_comm_res != COMM_SUCCESS:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getTxRxResult(dxl_comm_res)))
    elif dxl_err != 0:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getRxPacketError(dxl_err)))
    else:
        if (torque_mode == TORQUE_DISABLE):      print("SUCCESS: Torque is disabled")
        elif (torque_mode == TORQUE_ENABLE):     print("SUCCESS: Torque is enabled")
        
        
def DXL_read_torque_mode(dxl_id: int, port_handler, packet_handler) -> Optional[int]:
    val, dxl_comm_res, dxl_err = packet_handler.read1ByteTxRx(
        port_handler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
    )
    if dxl_comm_res != COMM_SUCCESS:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getTxRxResult(dxl_comm_res)))
        return None
    elif dxl_err != 0:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getRxPacketError(dxl_err)))
        return None
    else:
        torque_mode = val
        print(f"[DXL ID:{dxl_id}] SUCCESS: torque mode = {torque_mode}")
        return torque_mode    # 0: Disabled, 1: Enabled  


def DXL_write_operation_mode(dxl_id: int, operation_mode: int, port_handler, packet_handler) -> None:
    dxl_comm_res, dxl_err = packet_handler.write1ByteTxRx(
        port_handler,
        dxl_id,
        ADDR_OPERATING_MODE,
        int(operation_mode),
    )
    if dxl_comm_res != COMM_SUCCESS:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getTxRxResult(dxl_comm_res)))
    elif dxl_err != 0:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getRxPacketError(dxl_err)))
    else:
        if (operation_mode == CURRENT_CONTROL_MODE):      print("SUCCESS: Set Current Control Mode")
        elif (operation_mode == POSITION_CONTROL_MODE):   print("SUCCESS: Set Postion Control Mode")
    
    
def DXL_read_operation_mode(dxl_id: int, port_handler, packet_handler) -> Optional[int]:
    val, dxl_comm_res, dxl_err = packet_handler.read1ByteTxRx(
        port_handler,
        dxl_id,
        ADDR_OPERATING_MODE,
    )
    if dxl_comm_res != COMM_SUCCESS:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getTxRxResult(dxl_comm_res)))
        return None
    elif dxl_err != 0:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getRxPacketError(dxl_err)))
        return None
    else:
        operation_mode = val
        print(f"[DXL ID:{dxl_id}] SUCCESS: operation mode = {operation_mode}")
        return operation_mode    # 0: Current Control, 3: Position Control    

    

def DXL_read_position_rad(dxl_id: int, port_handler, packet_handler) -> Optional[float]:
    val, dxl_comm_res, dxl_err = packet_handler.read4ByteTxRx(
        port_handler,
        dxl_id,
        ADDR_PRESENT_POSITION,
    )
    if dxl_comm_res != COMM_SUCCESS:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getTxRxResult(dxl_comm_res)))
        return None
    elif dxl_err != 0:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getRxPacketError(dxl_err)))
        return None
    else:
        position_rad = float(val) / RESOLUTION_TICKS_PER_REV * TWO_PI 
        print(f"[DXL ID:{dxl_id}] SUCCESS: position = {position_rad}rad")
        return position_rad    # [rad]


# 1-unit = 2.69mA
def DXL_write_current_ampere(dxl_id: int, current_ampere: float, port_handler, packet_handler) -> None:        
    current_value = int(current_ampere / 2.69 * 1000.0)
    dxl_comm_res, dxl_err = packet_handler.write2ByteTxRx(
        port_handler,
        dxl_id,
        ADDR_GOAL_CURRENT,
        current_value,
    )
    if dxl_comm_res != COMM_SUCCESS:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getTxRxResult(dxl_comm_res)))
    elif dxl_err != 0:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getRxPacketError(dxl_err)))
    else:
        print(f"[DXL ID:{dxl_id}] SUCCESS: Set Current [{current_ampere} A]")
        

# 1-unit = 2.69mA / Ensure the connection
def DXL_write_current_ampere_safety(dxl_id: int, current_ampere: float, port_handler, packet_handler) -> None:        
    current_value = int(current_ampere / 2.69 * 1000.0)
    
    while True:
        dxl_comm_res, dxl_err = packet_handler.write2ByteTxRx(
            port_handler,
            dxl_id,
            ADDR_GOAL_CURRENT,
            current_value,
        )
        if dxl_comm_res != COMM_SUCCESS:
            print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getTxRxResult(dxl_comm_res)))
            time.sleep(1)
        elif dxl_err != 0:
            print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getRxPacketError(dxl_err)))
            time.sleep(1)
        else:
            print(f"[DXL ID:{dxl_id}] SUCCESS: Set Current [{current_ampere} A]")
            time.sleep(1)
            break
        print(f"[DXL ID]:{dxl_id} Retry...")
        
def DXL_current_control_smooth(dxl_id: int, port_handler, packet_handler, current_ampere_start=-0.03, current_ampere_end=0.0, sleep_time=1) -> None:
    current_value_start = int(current_ampere_start / 2.69 * 1000.0)
    current_value_end = int(current_ampere_end / 2.69 * 1000.0)
    
    state = 0
    
    if (state == 0):
        while True:
            dxl_comm_res, dxl_err = packet_handler.write2ByteTxRx(
                port_handler,
                dxl_id,
                ADDR_GOAL_CURRENT,
                current_value_start,
            )
            if dxl_comm_res != COMM_SUCCESS:
                print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getTxRxResult(dxl_comm_res)))
                time.sleep(1)
            elif dxl_err != 0:
                print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getRxPacketError(dxl_err)))
                time.sleep(1)
            else:
                print(f"[DXL ID:{dxl_id}] SUCCESS: Set Starting Current [{current_ampere_start} A]")
                state = 1
                time.sleep(sleep_time)
                break   
            print(f"[DXL ID]:{dxl_id} Retry starting current...")
            
    elif (state == 1):
        while True:
            dxl_comm_res, dxl_err = packet_handler.write2ByteTxRx(
                port_handler,
                dxl_id,
                ADDR_GOAL_CURRENT,
                current_value_end,
            )
            if dxl_comm_res != COMM_SUCCESS:
                print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getTxRxResult(dxl_comm_res)))
                time.sleep(1)
            elif dxl_err != 0:
                print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getRxPacketError(dxl_err)))
                time.sleep(1)
            else:
                print(f"[DXL ID:{dxl_id}] SUCCESS: Set End Current [{current_ampere_end} A]")
                time.sleep(1)
                break   
            print(f"[DXL ID]:{dxl_id} Retry End current...")


# 1-unit = 2.69mA
def DXL_read_current_ampere(dxl_id: int, port_handler, packet_handler) -> None:        
    val, dxl_comm_res, dxl_err = packet_handler.read2ByteTxRx(
        port_handler,
        dxl_id,
        ADDR_PRESENT_CURRENT
    )
    if dxl_comm_res != COMM_SUCCESS:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getTxRxResult(dxl_comm_res)))
        return None
    elif dxl_err != 0:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getRxPacketError(dxl_err)))
        return None
    else:
        current_A = float(val) * 2.69 / 1000.0 
        print(f"[DXL ID:{dxl_id}] SUCCESS: current = {current_A}A")
        return current_A     # [A]
    
    
# Range (0~1193)
def DXL_read_current_limit(dxl_id: int, port_handler, packet_handler) -> None:        
    val, dxl_comm_res, dxl_err = packet_handler.read2ByteTxRx(
        port_handler,
        dxl_id,
        ADDR_CURRENT_LIMIT
    )
    if dxl_comm_res != COMM_SUCCESS:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getTxRxResult(dxl_comm_res)))
        return None
    elif dxl_err != 0:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getRxPacketError(dxl_err)))
        return None
    else:
        current_limit_raw = val
        current_limit_A   = float(val) * 2.69 / 1000.0
        print(f"[DXL ID:{dxl_id}] SUCCESS: current_limit_raw(0~1193) = {current_limit_raw}, current_limit_A = {current_limit_A}A")
        return current_limit_A     # [A]
    

# Range (0~1193)
def DXL_write_current_limit(dxl_id: int, current_limit_A: float, port_handler, packet_handler) -> None:        
    current_limit_raw = int(current_limit_A / 2.69 * 1000.0)
    if  (current_limit_raw > 1193):     current_limit_raw = 1193

    dxl_comm_res, dxl_err = packet_handler.write2ByteTxRx(
        port_handler,
        dxl_id,
        ADDR_CURRENT_LIMIT,
        current_limit_raw
    )
    if dxl_comm_res != COMM_SUCCESS:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getTxRxResult(dxl_comm_res)))
    elif dxl_err != 0:
        print("[DXL ID:%d] Fail %s" % (dxl_id, packet_handler.getRxPacketError(dxl_err)))
    else:
        print(f"SUCCESS: Set Current Limit: {current_limit_A}A")
    




#----- Functions for IntelRealsense(D435i) --------------------#
def find_block_center(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 180])
    upper_white = np.array([180, 60, 255])
    white_mask = cv2.inRange(hsv, lower_white, upper_white)

    kernel = np.ones((7, 7), np.uint8)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None, None

    table = max(contours, key=cv2.contourArea)

    table_mask = np.zeros_like(white_mask)
    cv2.drawContours(table_mask, [table], -1, 255, -1)

    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 70])
    black_mask = cv2.inRange(hsv, lower_black, upper_black)

    block_mask = cv2.bitwise_and(black_mask, table_mask)
    block_mask = cv2.morphologyEx(block_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(
        block_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 300 < area < 20000:
            candidates.append(cnt)

    if not candidates:
        return None, block_mask

    block = max(candidates, key=cv2.contourArea)

    M = cv2.moments(block)
    if M["m00"] == 0:
        return None, block_mask

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    return (cx, cy, block), block_mask


def get_block_position_5sec():
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    pipeline.start(config)
    align = rs.align(rs.stream.color)

    X_list, Y_list, Z_list = [], [], []

    start_time = time.time()
    
    cnt = 0

    try:
        while time.time() - start_time < 5:  # 5초 동안만 실행
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())

            result, _ = find_block_center(frame)

            if result is None:
                continue

            cx, cy, _ = result
            z = depth_frame.get_distance(cx, cy)

            if z <= 0:
                continue

            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

            X, Y, Z = rs.rs2_deproject_pixel_to_point(
                depth_intrin,
                [cx, cy],
                z
            )

            if (cnt % 20 == 0):
                print(f"X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}")
            cnt = cnt + 1

            X_list.append(X)
            Y_list.append(Y)
            Z_list.append(Z)

    finally:
        pipeline.stop()

    # 평균 계산
    if len(X_list) == 0:
        print("Error! No value is acquired")
        return None

    X_avg = np.mean(X_list)
    Y_avg = np.mean(Y_list)
    Z_avg = np.mean(Z_list)

    print("\n===== Average =====")
    print(f"X={X_avg:.3f}, Y={Y_avg:.3f}, Z={Z_avg:.3f}")

    return X_avg, Y_avg, Z_avg