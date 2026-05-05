import rby1_sdk as rby
from dynamixel_sdk import PortHandler, PacketHandler
import time
import numpy as np
import functions as fn

# Before run this file, run the "initialize.py" file
# ----- Connect -------------------------------------------------------------------#
ROBOT_ADDRESS = "192.168.30.1:50051"        # For Real Robot
robot = rby.create_robot_a(ROBOT_ADDRESS)   # Robot Model is "A"
model = robot.model()

if not robot.connect():
    print("Could not connect to robot")
    exit(1)
else:
    print("SUCCESS: Connect to the robot.")



# ----- Open Gripper ports -------------------------------------------------------#
port_hdlr = PortHandler(fn.DEVICE_NAME_GRIPPER)
packet_hdlr = PacketHandler(fn.PROTOCOL_VERSION)

if port_hdlr.openPort():
    print(f"SUCCESS: open the port: {fn.DEVICE_NAME_GRIPPER}")
else:
    raise RuntimeError(f"Failed to open the port: {fn.DEVICE_NAME_GRIPPER}")

if port_hdlr.setBaudRate(fn.BAUDRATE):
    print(f"SUCCESS: change the baudrate {fn.BAUDRATE}")
else:
    raise RuntimeError(f"Failed to change the baudrate")

# --- Ping Test ---
dxl_activated = []

for dxl_id in [fn.R_DXL_ID, fn.L_DXL_ID]:
    dxl_model_num, dxl_comm_res, dxl_err = packet_hdlr.ping(port_hdlr, dxl_id)
    
    if dxl_comm_res != fn.COMM_SUCCESS:
        print("[DXL ID:%d] %s" % (dxl_id, packet_hdlr.getTxRxResult(dxl_comm_res)))
    elif dxl_err != 0:
        print("[DXL ID:%d] %s" % (dxl_id, packet_hdlr.getRxPacketError(dxl_err)))
    else:
        dxl_activated.append(dxl_id)
        print("[DXL ID:%d] ping Succeeded. Dynamixel model number : %d" % (dxl_id, dxl_model_num))

# --- Enable Current Control ---
for dxl_id in dxl_activated:
    fn.DXL_write_torque_mode(dxl_id, fn.TORQUE_DISABLE, port_hdlr, packet_hdlr)             # Operation mode 변경 전 torque OFF
    fn.DXL_write_current_limit(dxl_id, fn.CURRENT_LIMIT_MAX_A, port_hdlr, packet_hdlr)      # Current limit setting before torque ON
    fn.DXL_write_operation_mode(dxl_id, fn.CURRENT_CONTROL_MODE, port_hdlr, packet_hdlr)
    fn.DXL_write_torque_mode(dxl_id, fn.TORQUE_ENABLE, port_hdlr, packet_hdlr)
        

# ----- Ready Initial Posture --------------------------------------------------- #
Desired_Pos = fn.READY_DEMO_POS      # ZERO_POS, READY_POS, PACK_POS, READY_DEMO_POS
fn.move_position(robot, Desired_Pos, minimum_time=3)
time.sleep(1)
fn.DXL_write_current_ampere_safety(fn.L_DXL_ID, fn.DEMO_ZEROING_CURRENT_L, port_hdlr, packet_hdlr)


# ----- Go Forward to 1st Table --------------------------------------------------- #
fn.move_forward_distance_smooth(robot, 0.31)         # Forward 30cm
time.sleep(2)


# ----- Move Left arm to Left Block --------------------------------------------------- #
# pos_delta_from_torso_R = [0.175, 0.14, 0.0]
pos_delta_from_torso_R = [0.0, 0.0, 0.0]
pos_delta_from_torso_L = [0.18, -0.1, 0.0]
fn.move_arm_globalpos_both(robot, pos_delta_from_torso_R, pos_delta_from_torso_L)

time.sleep(1)
# fn.DXL_current_control_smooth(fn.R_DXL_ID, port_hdlr, packet_hdlr, fn.DEMO_LAY_CURRENT_R, fn.DEMO_LAY_CURRENT_END)
fn.DXL_current_control_smooth(fn.L_DXL_ID, port_hdlr, packet_hdlr, fn.DEMO_LAY_CURRENT_L, fn.DEMO_LAY_CURRENT_END)

z = -0.16
# pos_delta_from_torso_R = [0.0, 0.0, z]
pos_delta_from_torso_R = [0.0, 0.0, 0.0]
pos_delta_from_torso_L = [0.0, 0.0, z]
fn.move_arm_globalpos_both(robot, pos_delta_from_torso_R, pos_delta_from_torso_L)
time.sleep(1)


# ----- Catch the Block --------------------------------------------------- #
# fn.DXL_write_current_ampere_safety(fn.R_DXL_ID, fn.DEMO_GRIP_CURRENT_R, port_hdlr, packet_hdlr)
fn.DXL_write_current_ampere_safety(fn.L_DXL_ID, fn.DEMO_GRIP_CURRENT_L, port_hdlr, packet_hdlr)


# ----- Lift UP ------------------------------------------------------------ #
z = 0.2
# pos_delta_from_torso_R = [0.0, 0.0, z]
pos_delta_from_torso_R = [0.0, 0.0, 0.04]
pos_delta_from_torso_L = [0.0, 0.0, z]
fn.move_arm_globalpos_both(robot, pos_delta_from_torso_R, pos_delta_from_torso_L)
time.sleep(1)


# ----- Go Backward from 1st Table --------------------------------------------------- #
fn.move_forward_distance_smooth(robot, -0.31)         # Backward 30cm
time.sleep(1)


# ----- Turn Right--------------------------------------------------- #
fn.rotate_inplace_smooth(robot, fn.TURN_RIGHT_90)                # Right turn 90degree
time.sleep(1)


# ----- Go Forward to 2nd Table --------------------------------------------------- #
fn.move_forward_distance_smooth(robot, 2.5)         # Forward 250cm
time.sleep(1)


# ----- Turn Left --------------------------------------------------- #
fn.rotate_inplace_smooth(robot, fn.TURN_LEFT_90)                # Left turn 90degree
time.sleep(1)


# ----- Go Forward to 2nd Table --------------------------------------------------- #
fn.move_forward_distance_smooth(robot, 0.31)         # Forward 30cm
time.sleep(1)


# ----- Move Left arm to Left collecting box -------------------------------------------- #
# pos_delta_from_torso_R = [0.0, -0.17, 0.0]
pos_delta_from_torso_R = [0.0, 0.0, 0.0]
pos_delta_from_torso_L = [0.0, 0.13, 0.0]
fn.move_arm_globalpos_both(robot, pos_delta_from_torso_R, pos_delta_from_torso_L)
time.sleep(1)

z = -0.08
# pos_delta_from_torso_R = [0.0, 0.0, z]
pos_delta_from_torso_R = [0.0, 0.0, 0.0]
pos_delta_from_torso_L = [0.0, 0.0, z]
fn.move_arm_globalpos_both(robot, pos_delta_from_torso_R, pos_delta_from_torso_L)
time.sleep(1)


# ----- Lay block to Left box --------------------------------------------------- #
# fn.DXL_current_control_smooth(fn.R_DXL_ID, port_hdlr, packet_hdlr, fn.DEMO_LAY_CURRENT_R, fn.DEMO_LAY_CURRENT_END)
fn.DXL_current_control_smooth(fn.L_DXL_ID, port_hdlr, packet_hdlr, fn.DEMO_LAY_CURRENT_L, fn.DEMO_LAY_CURRENT_END)


# ----- Lift up Left arm --------------------------------------------------- #
z = 0.08
# pos_delta_from_torso_R = [0.0, 0.0, z]
pos_delta_from_torso_R = [0.0, 0.0, 0.0]
pos_delta_from_torso_L = [0.0, 0.0, z]
fn.move_arm_globalpos_both(robot, pos_delta_from_torso_R, pos_delta_from_torso_L)
time.sleep(1)


# ----- Go Backward from 2nd Table --------------------------------------------------- #
fn.move_forward_distance_smooth(robot, -0.31)         # Backward 30cm
time.sleep(1)


# ----- Turn Left --------------------------------------------------- #
fn.rotate_inplace_smooth(robot, fn.TURN_LEFT_90)                # Left turn 90degree
time.sleep(1)


# ----- Go Forward to 1st Table --------------------------------------------------- #
fn.move_forward_distance_smooth(robot, 2.5)         # Forward 250cm
time.sleep(1)


# ----- Turn Right --------------------------------------------------- #
fn.rotate_inplace_smooth(robot, fn.TURN_RIGHT_90)                # Right turn 90degree
time.sleep(1)




































