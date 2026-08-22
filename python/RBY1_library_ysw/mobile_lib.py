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
    distance_m,         
    max_speed=0.25,     
    accel=0.05,
    tolerance=0.01,     
    dt=0.02,
    yaw_hold_kp=1.0,     
    max_yaw_rate_for_hold=0.15,  
    stop_speed_eps=0.005,        
    verbose=True
):
    """
        Move forward/backward by distance_m using odometry feedback.

        distance_m > 0 : forward
        distance_m < 0 : backward
        
        Args:
            distance_m: desired moved distance, unit=[m]
            max_speed: allowable maximum speed, unit=[m/s]
            accel: used acceleration, unit=[m/s^2]
            tolerance: distance error tolerance, unit=[m]
            yaw_hold_kp: P-gain for yaw maintenance
            max_yaw_rate_for_hold: maximum value for control yaw_rate  
            stop_speed_eps: Stop algorithm is conducted until reach this value
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

            # 실제 얼마나 이동했는지 (forward 방향만 추출)
            raw_forward_displacement = float(np.dot(delta_pos, forward_axis_world))

            # 제어용: 목표 방향으로 얼마나 진행했는지
            moved_distance = move_sign * raw_forward_displacement
            # 출력용: forward는 +, backward는 -
            signed_moved_distance = raw_forward_displacement
            
            error = target_distance - moved_distance
            
            # yaw 유지 보정 (yaw 틀어짐 방지)
            yaw_error = normalize_angle(yaw - yaw0)
            yaw_rate_cmd = clamp(
                -yaw_hold_kp * yaw_error,
                -max_yaw_rate_for_hold,
                max_yaw_rate_for_hold,
            )
            
            # =========================================================
            # [STOP phase]
            # If close enough to target, smoothly reduce commanded speed
            # =========================================================
            if error <= tolerance:
                desired_v_x = 0.0

                max_dv = accel * dt
                v_x_cmd += clamp(
                    desired_v_x - v_x_cmd,
                    -max_dv,
                    max_dv,
                )

                cmd = build_mobile_cmd(
                    v_x=v_x_cmd,
                    v_y=0.0,
                    yaw_rate=0.0,
                    hold_time=0.25,
                    acc=accel,
                )

                stream.send_command(cmd, timeout_ms=500)

                if verbose and time.time() - last_print_time > 0.2:
                    print(
                        f"[Stopping] "
                        f"distance={signed_moved_distance:.3f}[m], "
                        f"speed={v_x_cmd:.3f}[m/s], "
                        f"pos_error={error:.3f}[m]"
                    )
                    last_print_time = time.time()

                if abs(v_x_cmd) <= stop_speed_eps:
                    return True

                time.sleep(dt)
                continue


            # =========================================================
            # [Normal movement phase]
            # =========================================================
            
            # 남은 거리 기준으로 안전하게 감속 가능한 속도
            # v^2 = 2 a x
            desired_speed_mag = np.sqrt(max(0.0, 2.0 * accel * max(0.0, error)))
            desired_speed_mag = min(max_speed, desired_speed_mag)

            # 목표 방향 속도
            desired_v_x = move_sign * desired_speed_mag
            
            # 속도 변화량 제한
            max_dv = accel * dt
            v_x_cmd += clamp(desired_v_x - v_x_cmd, -max_dv, max_dv)

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
                    f"distance={signed_moved_distance:.3f}[m], "
                    f"speed={v_x_cmd:.3f}[m/s], "
                    f"pos_error={error:.3f}[m], "
                    f"yaw_error={np.rad2deg(yaw_error):.2f}[deg]"
                )
                last_print_time = time.time()

            time.sleep(dt)

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
     

        
        
        
        
        
        
        
        
        