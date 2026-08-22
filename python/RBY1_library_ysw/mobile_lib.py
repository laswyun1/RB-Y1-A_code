# Mobile(Wheel movement) Library for RBY1-A
# This Library contains Mobile Movement functions 
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
import time
from typing import Sequence


# =============================================================================
# Functions
# =============================================================================

def clamp(x, x_min, x_max):
    return max(x_min, min(x_max, x))



# Angle unit = [rad]
def normalize_angle(angle_rad):
    """
        Make the angle range [-pi, +pi)
    """
    return (angle_rad + np.pi) % (2 * np.pi) - np.pi



def get_odom_2Dpose(robot):
    """
        The return value of "robot.get_state().odometry" is SE(2) 3x3 2D homogemeous transform
        
        Example: 
            [[ cos(theta), -sin(theta), x ],
            [ sin(theta),  cos(theta), y ],
            [    0.0,          0.0,    1.0 ]]
            
        This represents T_odom_base.
        {odom} frame = {base} frame at initial state or when the "robot.reset_odometry()" is called
        
        {odom} frame:
            Robot's internal odometry frame.
            If reset_odometry() is not called, this frame is whatever the robot
            has been using since boot, controller start, or previous reset.
            
        +x: Forward direction of robot
        +y: Right direction of robot
        
        return:
            pos_x: {base} position x in {odom} frame [m]
            pos_y: {base} position y in {odom} frame [m]
            ori_yaw: {base} yaw in {odom} frame [rad]
    """
    odom = robot.get_state().odometry

    pos_x = float(odom[0, 2])
    pos_y = float(odom[1, 2])
    ori_yaw = float(np.arctan2(odom[1, 0], odom[0, 0]))
    
    return pos_x, pos_y, ori_yaw



def build_mobile_cmd(v_x=0.0, v_y=0.0, yaw_rate=0.0, hold_time=0.2, acc=0.2):
    return rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_mobility_command(
            rby.SE2VelocityCommandBuilder()
            .set_velocity(np.array([v_x, v_y], dtype=np.float64), float(yaw_rate))
            .set_acceleration_limit(np.array([acc, acc], dtype=np.float64), float(acc))
            .set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(hold_time)
            )
        )
    )
    
    
def stop_mobile(stream, acc=0.2, repeat=8):
    """
        Send zero velocity commands repeatedly to stop mobile base safely.
    """
    for _ in range(repeat):
        cmd = build_mobile_cmd(
            v_x=0.0, 
            v_y=0.0, 
            yaw_rate=0.0, 
            hold_time=0.3, 
            acc=acc
        )
        stream.send_command(cmd, timeout_ms=1000)
        time.sleep(0.03)
        
        

def mobile_linear_distance(
    robot,
    distance_m,          # Unit = [m]
    max_speed=0.25,      # Unit = [m/s]
    min_speed=0.02,
    accel=0.10,
    tolerance=0.01,
    dt=0.02,
    yaw_hold_kp=1.0,    # P-gain for yaw maintenance
    max_yaw_rate_for_hold=0.15,
    verbose=True
):
    """
        Move forward/backward by distance_m using odometry feedback.

        distance_m > 0 : forward
        distance_m < 0 : backward
    """  

    if abs(distance_m) <= tolerance:
        return True
    
    x0, y0, yaw0 = get_odom_2Dpose(robot)
    
    # "yaw at start time"-based forward direction
    forward_axis_world = np.array([np.cos(yaw0), np.sin(yaw0)], dtype=np.float64)
    
    move_sign = 1.0 if distance_m >= 0.0 else -1.0     # +1: forward, -1: backward
    target_distance = abs(distance_m)
    
    v_x_cmd = 0.0
    last_print_time = 0.0
    
    stream = None
    
    try:
        stream = robot.create_command_stream(priority=1)
        
        while True:
            x, y, yaw = get_odom_2Dpose(robot)

            delta_pos = np.array([x - x0, y - y0], dtype=np.float64)

            # 목표 방향으로 실제 얼마나 이동했는지 (forward 방향만 추출)
            moved_distance = move_sign * float(np.dot(delta_pos, forward_axis_world))

            error = target_distance - moved_distance

            if abs(error) <= tolerance:
                # stop_mobile(stream, acc=accel)    # finally에서 어차피 실행되므로 주석처리 가능
                return True

            # 남은 거리 기준으로 안전하게 감속 가능한 속도
            # v^2 = 2 a x
            desired_speed_mag = np.sqrt(max(0.0, 2.0 * accel * abs(error)))
            desired_speed_mag = min(max_speed, desired_speed_mag)

            # 너무 작은 속도는 실제 로봇에서 동작되지 않을 수 있으므로 최소 속도 부여
            if desired_speed_mag < min_speed:
                desired_speed_mag = min_speed

            # overshoot되면 반대 방향으로 보정
            desired_v_x = move_sign * np.sign(error) * desired_speed_mag

            # 속도 변화량 제한
            max_dv = accel * dt
            v_x_cmd += clamp(desired_v_x - v_x_cmd, -max_dv, max_dv)

            # yaw 유지 보정 (yaw 틀어짐 방지)
            yaw_error = normalize_angle(yaw - yaw0)
            yaw_rate_cmd = clamp(
                -yaw_hold_kp * yaw_error,
                -max_yaw_rate_for_hold,
                max_yaw_rate_for_hold,
            )

            cmd = build_mobile_cmd(
                v_x=v_x_cmd,
                v_y=0.0,
                yaw_rate=yaw_rate_cmd,
                hold_time=0.25,
                acc=accel,
            )

            # Important: .get()으로 매번 기다리지 않고 stream으로 연속 전송
            stream.send_command(cmd, timeout_ms=500)

            if verbose and time.time() - last_print_time > 0.2:
                print(
                    f"moved_distance={moved_distance:.3f}[m], "
                    f"speed={v_x_cmd:.3f}[m/s], "
                    f"pos_error={error:.3f}[m], "
                    f"yaw_error={np.rad2deg(yaw_error):.2f}[deg]"
                )
                last_print_time = time.time()

            time.sleep(dt)

    except KeyboardInterrupt:
        logging.warning("[mobile_linear_distance] interrupted by user.")
        return False

    except Exception as e:
        logging.error(f"[mobile_linear_distance] failed: {e}")
        return False

    # For Emergency, Make sure STOP
    finally:
        if stream is not None:
            try:
                stop_mobile(stream, acc=accel)
            except Exception as e:
                logging.warning(f"Failed to [stop_mobile]: {e}")

            try:
                stream.cancel()
            except Exception as e:
                logging.warning(f"Failed to cancel [mobile stream]: {e}")
     

        
        
        
        
        
        
        
        
        