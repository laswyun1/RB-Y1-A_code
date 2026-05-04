import rby1_sdk as rby
import time
import numpy as np


ROBOT_ADDRESS = "192.168.30.1:50051"        # For Real Robot
RIGHT_END     = 0       # Right End Effector idx
LEFT_END      = 1       # Left End Effector idx

ZERO_POS = {"torso":      np.deg2rad([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
             "right_arm": np.deg2rad([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
             "left_arm":  np.deg2rad([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])}

READY_POS = {"torso":     np.deg2rad([0.0, 45.0, -90.0,   45.0, 0.0,  0.0]),
            "right_arm":  np.deg2rad([0.0, -5.0,   0.0, -120.0, 0.0, 70.0, 0.0]),
            "left_arm":   np.deg2rad([0.0,  5.0,   0.0, -120.0, 0.0, 70.0, 0.0])}

PACK_POS = {"torso":      np.deg2rad([0.0, 80.0, -140.0,   60.0, 0.0,   0.0]),
            "right_arm":  np.deg2rad([0.0,  0.0,    0.0, -150.0, 0.0, 105.0, 0.0]),
            "left_arm":   np.deg2rad([0.0,  0.0,    0.0, -150.0, 0.0, 105.0, 0.0])}


def move_position(robot, posture, minimum_time=10.0):
    rc_builder = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            # Set Torso Command with Joint Position Command
            .set_torso_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(0.5))  # For saturation
                .set_minimum_time(minimum_time)
                .set_position(posture["torso"])
            )
            # Set Right Command with Joint Position Command
            .set_right_arm_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(0.5))
                .set_minimum_time(minimum_time)
                .set_position(posture["right_arm"])
            )
            # Set Left Command with Joint Position Command
            .set_left_arm_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(0.5))
                .set_minimum_time(minimum_time)
                .set_position(posture["left_arm"])
            )
        )
    )
    rc = robot.send_command(rc_builder).get()
    return rc.finish_code == rby.RobotCommandFeedback.FinishCode.Ok



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
    max_speed=0.5,       # m/s
    accel=0.2,           # m/s^2, 작을수록 천천히 가속
    tolerance=0.01,
    dt=0.02              # 50 Hz
):
    robot.reset_odometry(
        angle=0.0,
        position=np.array([0.0, 0.0])
    )

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
    max_yaw_rate=0.5,
    min_yaw_rate=0.1,
    accel=0.2,
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
def move_endeffector(robot, direction_ee, rel_delta_pose):
    dyn_model = robot.get_dynamics()
    dyn_state = dyn_model.make_state(["base", "link_torso_5", "ee_right", "ee_left"], robot.model().robot_joint_names)
    BASE_INDEX, LINK_TORSO_5_INDEX, EE_RIGHT_INDEX, EE_LEFT_INDEX = 0, 1, 2, 3

    dyn_state.set_q(robot.get_state().position)         # Get current gc
    dyn_model.compute_forward_kinematics(dyn_state)     # FK
    
    T_delta = np.eye(4)
    T_delta[0:3, 3] = rel_delta_pose
    
    if (direction_ee == RIGHT_END):
        T_torso2ee = dyn_model.compute_transformation(dyn_state, LINK_TORSO_5_INDEX, EE_RIGHT_INDEX)
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
        T_torso2ee = dyn_model.compute_transformation(dyn_state, LINK_TORSO_5_INDEX, EE_LEFT_INDEX)
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