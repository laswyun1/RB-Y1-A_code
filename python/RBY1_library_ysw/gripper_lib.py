# Default Library for RBY1-A Grippers(Dynamixel XM430-W210)
# This Library contains robot default settings and general functions
# This should be imported in "rby1_lib_ysw.py"
# 
# Usage example:
#       from default_lib import *
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

from dynamixel_sdk import PortHandler, PacketHandler
from typing import Optional
import time
import matplotlib.pyplot as plt
import numpy as np
from IPython.display import display


# =============================================================================
# Code
# =============================================================================
class Gripper:
    def __init__(self):
        """
            Create Gripper object 
            [Ex] gripper = ysw.Gripper()
        """
        self.port_hdlr = None 
        self.packet_hdlr = None
        self.dxl_R_idx = 0
        self.dxl_L_idx = 1
        self.dxl_activated = []
        
        self.dxl_R_homing_openTick     = RI.GRIPPER_OPEN_POS_RAWTICK_R
        self.dxl_L_homing_openTick     = RI.GRIPPER_OPEN_POS_RAWTICK_L
        self.dxl_R_homing_closeTick    = RI.GRIPPER_CLOSE_POS_RAWTICK_R
        self.dxl_L_homing_closeTick    = RI.GRIPPER_CLOSE_POS_RAWTICK_L
        self.dxl_R_homing_rangeTick    = RI.GRIPPER_POS_RANGE_TICK_R
        self.dxl_L_homing_rangeTick    = RI.GRIPPER_POS_RANGE_TICK_L
        self.dxl_R_homing_LinearToTick = RI.GRIPPER_POS_SCALE_LINEAR_TO_TICK_R
        self.dxl_L_homing_LinearToTick = RI.GRIPPER_POS_SCALE_LINEAR_TO_TICK_L
        
        self.command_time = None        # command input time (ex. position control)
        
        
        
    #----------------------------------------------------------------------------
    # Essential Functions (Must be used after creating Gripper Object)
    # ---------------------------------------------------------------------------
    def connect(self, device_name="/dev/rby1_gripper", protocol_version=2.0, baudrate=2000000) -> bool:
        """
            Connection function (Essential)
            [Ex] gripper.connect(device_name=ysw.GRIPPER_DEVICE_NAME, protocol_version=ysw.DXL_PROTOCOL_VERSION, baudrate=ysw.DXL_BAUDRATE)
                 
            Return: 
                True: connected
                False: not connected
        """
        UTIL.print_start_title("Gripper Connection Starts")
        
        self.port_hdlr = PortHandler(device_name)
        self.packet_hdlr = PacketHandler(protocol_version)
        
        if self.port_hdlr.openPort():
            logging.info(f"SUCCESS: open the port: {device_name}")
            if self.port_hdlr.setBaudRate(baudrate):
                logging.info(f"SUCCESS: change the baudrate {baudrate}")
                UTIL.print_end_title("Gripper Connection is Done")
                return True
            else:
                logging.error(f"Failed to change the baudrate {baudrate}")
                return False
        else:
            logging.error(f"Failed to open the port: {device_name}")
            return False



    def test_ping(self) -> bool:
        """
            Connection Test function (Optioinal)
            [Ex] gripper.test_ping()
            
            Return: 
                True: ping test OK
                False: ping test error
        """
        UTIL.print_start_title("Gripper Ping Test Starts")
        self.dxl_activated = []
        
        for dxl_id in [RI.GRIPPER_ID_R, RI.GRIPPER_ID_L]:
            dxl_model_num, dxl_comm_res, dxl_err = self.packet_hdlr.ping(self.port_hdlr, dxl_id)
            
            if dxl_comm_res != RI.DXL_COMM_SUCCESS:
                logging.error("[DXL ID:%d] %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
                return False
            elif dxl_err != RI.DXL_NO_ERROR:
                logging.error("[DXL ID:%d] %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
                return False
            else:
                self.dxl_activated.append(dxl_id)
                logging.info("[DXL ID:%d] SUCCESS: Dynamixel model number is %d" % (dxl_id, dxl_model_num))
                
        UTIL.print_end_title("Gripper Ping Test is Done")
        return True
    
    
    def connect_hardCheck(
        self, 
        device_name="/dev/rby1_gripper", 
        protocol_version=2.0, 
        baudrate=2000000,
        retry_interval=1.0,
        max_retry=5
    ) -> bool:
        """
            Connection function (Strict version): "connect" + "test_ping"

            This function waits until both grippers are connected and ping test passes.

            Args:
                device_name: Dynamixel device path
                protocol_version: Dynamixel protocol version
                baudrate: Dynamixel baudrate
                retry_interval: waiting time between retries [s]
                max_retry:
                    None: retry forever
                    int : retry up to max_retry times

            Return:
                True: connection and ping test succeeded
                False: failed after max_retry
            
            [Ex] gripper.connect_hardCheck(
                    device_name=ysw.GRIPPER_DEVICE_NAME, 
                    protocol_version=ysw.DXL_PROTOCOL_VERSION, 
                    baudrate=ysw.DXL_BAUDRATE,
                )
        """
        UTIL.print_start_title("Gripper Hard Connection Starts")

        attempt = 0

        while True:
            attempt += 1

            print(f"[Gripper] Trying to connect gripper... attempt={attempt}")

            rv_connect = self.connect(
                device_name=device_name,
                protocol_version=protocol_version,
                baudrate=baudrate,
            )

            if rv_connect:
                rv_ping = self.test_ping()
            else:
                rv_ping = False

            print(f"[Gripper] connect={rv_connect}, ping_test={rv_ping}")

            if rv_connect and rv_ping:
                UTIL.print_end_title("Gripper Hard Connection is Done")
                return True

            # If connection opened but ping failed, close port before retry
            try:
                if self.port_hdlr is not None:
                    self.port_hdlr.closePort()
                    logging.warning("[Gripper] Port closed before retry.")
            except Exception as e:
                logging.warning(f"[Gripper] Failed to close port before retry: {e}")

            if max_retry is not None and attempt >= max_retry:
                logging.error("[Gripper] Hard connection failed: max retry reached.")
                return False

            time.sleep(retry_interval)
                
                
                
    #----------------------------------------------------------------------------
    # Register Read Functions
    # ---------------------------------------------------------------------------
    def read_torque_mode(self, dxl_id: int) -> Optional[int]:
        """
            Read Torque Mode State 
            [Ex] gripper.read_torque_mode(dxl_id=ysw.GRIPPER_ID_R)
            
            Return:
                0: Disabled
                1: Enabled
        """  
        val, dxl_comm_res, dxl_err = self.packet_hdlr.read1ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_TORQUE_ENABLE
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return None
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return None
        else:
            torque_mode = val
            logging.info(f"[DXL ID:{dxl_id}] SUCCESS: torque mode = {torque_mode}")
            return torque_mode    # 0: Disabled, 1: Enabled   
        
        
        
    def read_operation_mode(self, dxl_id: int) -> Optional[int]:
        """
            Read Operation Mode State  
            [Ex] gripper.read_operation_mode(dxl_id=ysw.GRIPPER_ID_R)
            
            Return:
                0: Current Control
                3: Position Control
                4: Extended Position Control
                5: Current-based Position Control
        """  
        val, dxl_comm_res, dxl_err = self.packet_hdlr.read1ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_OPERATING_MODE
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return None
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return None
        else:
            operation_mode = val
            logging.info(f"[DXL ID:{dxl_id}] SUCCESS: operation mode = {operation_mode}")
            return operation_mode    # 0: Current Control, 3: Position Control, 5: Current-based Position Control  
        
        
        
    def read_position(self, dxl_id: int, unit=RI.RAD, homing_use=True, verbose=True) -> Optional[float]:
        """
            Read current position  
            Default Gripper Range(Approximation): -170deg(open) ~ +90deg(closed) 
            [Ex] gripper.read_position(dxl_id=ysw.GRIPPER_ID_R, unit=ysw.DEG, print_log=True)
            
            Return:
                angle position (unit = radian / degree / tick / mm)
        """  
        val, dxl_comm_res, dxl_err = self.packet_hdlr.read4ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_PRESENT_POSITION
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return None
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return None
        else:
            # Transform to signed 32-bit
            signed_val = val
            if signed_val >= 2**31:
                signed_val -= 2**32
                
            # Make Maximum Value(closed state) = 0-tick
            if homing_use:
                if dxl_id == RI.GRIPPER_ID_R:       
                    offset_tick = RI.GRIPPER_CLOSE_POS_RAWTICK_R 
                    mm_per_tick_scale =  1.0 / float(RI.GRIPPER_POS_SCALE_LINEAR_TO_TICK_R)       
                elif dxl_id == RI.GRIPPER_ID_L:     
                    offset_tick = RI.GRIPPER_CLOSE_POS_RAWTICK_L  
                    mm_per_tick_scale =  1.0 / float(RI.GRIPPER_POS_SCALE_LINEAR_TO_TICK_L)                
                signed_val -= offset_tick
                
            position_tick = signed_val
            position_rad  = float(signed_val) / RI.DXL_RESOLUTION_TICKS_PER_REV * RI.TWO_PI
            position_deg  = position_rad * RI.RAD2DEG
            if homing_use:
                position_mm = position_tick * mm_per_tick_scale
                            
            # Unit Selection
            if (unit == RI.RAD):
                if verbose == True: 
                    logging.info(f"SUCCESS: [DXL ID:{dxl_id}] position = {position_rad} rad")
                return position_rad    # [rad]
            elif (unit == RI.DEG):
                if verbose == True:
                    logging.info(f"SUCCESS: [DXL ID:{dxl_id}] position = {position_deg} deg")
                return position_deg    # [deg]
            elif (unit == RI.TICK):
                if verbose == True:
                    logging.info(f"SUCCESS: [DXL ID:{dxl_id}] position = {position_tick} tick")
                return position_tick    # [tick]
            elif (unit == RI.MM):
                if (homing_use == True):
                    if verbose == True:
                        logging.info(f"SUCCESS: [DXL ID:{dxl_id}] position = {position_mm} mm")
                    return position_mm    # [mm]   
                else:
                    logging.error("For [mm] unit, homing must be used")
                    return None
            
            
            
    def read_current_ampere(self, dxl_id: int, verbose=True) -> Optional[float]:    
        """
            Read Current value (unit = A) 
            [Ex] gripper.read_current_ampere(dxl_id=ysw.GRIPPER_ID_R)
            
            Return:
                current value (unit = ampere)
        """    
        val, dxl_comm_res, dxl_err = self.packet_hdlr.read2ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_PRESENT_CURRENT
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return None
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return None
        else:
            # Transform to signed 16-bit
            signed_val = val
    
            if signed_val >= 2**15:
                signed_val -= 2**16
                
            current_A = float(signed_val) * RI.DXL_AMPERE_PER_UNIT 
    
            if (verbose == True):
                logging.info(f"SUCCESS: [DXL ID:{dxl_id}] current = {current_A}A")
                
            return current_A     # [A]
        
        
        
    def read_current_limit(self, dxl_id: int) -> Optional[float]:  
        """
            Read Current Limit(MaX) value (unit = A) 
            Raw value Range: (0~1193)
            [Ex] gripper.read_current_limit(dxl_id=ysw.GRIPPER_ID_R)
            
            Return:
                Max Limit Current value (unit = ampere)
        """     
        val, dxl_comm_res, dxl_err = self.packet_hdlr.read2ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_CURRENT_LIMIT
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return None
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return None
        else:
            current_limit_raw = val
            current_limit_A   = float(val) * 2.69 / 1000.0
            logging.info(f"SUCCESS: [DXL ID:{dxl_id}] current_limit_raw(0~1193) = {current_limit_raw}, current_limit_A = {current_limit_A}A")
            return current_limit_A     # [A]     
        
        
        
    def read_PIDgain(self, dxl_id: int) -> list[float]:  
        """
            Read PID gain of present control(operation) mode
            Raw value Range: (0 ~ 16,383)
            [Ex] gripper.read_PIDgain(dxl_id=ysw.GRIPPER_ID_R)
            
            Return:
                List such that [Kp, Ki, Kd]
        """     
        val, dxl_comm_res, dxl_err = self.packet_hdlr.read2ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_P_GAIN_POSCTRL
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return None
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return None
        else:
            Kp = val
            logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Kp = {Kp}")
            
        val, dxl_comm_res, dxl_err = self.packet_hdlr.read2ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_I_GAIN_POSCTRL
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return None
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return None
        else:
            Ki = val
            logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Ki = {Ki}")
            
        val, dxl_comm_res, dxl_err = self.packet_hdlr.read2ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_D_GAIN_POSCTRL
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return None
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return None
        else:
            Kd = val
            logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Kd = {Kd}")
            return [Kp, Ki, Kd]
        
        

    def read_position_trajectory(self, dxl_id: int) -> Optional[float]:  
        """
            Read position trajectory 
            [Ex] gripper.read_position_trajectory(dxl_id=ysw.GRIPPER_ID_R)
            
            Return:
                
        """     
        val, dxl_comm_res, dxl_err = self.packet_hdlr.read2ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_POSITION_TRAJECTORY
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return None
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return None
        else:
            position_trajectory = val
            logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Position Trajectory = {position_trajectory}")
            return position_trajectory       
        
        
        
    def read_moving_status(self, dxl_id: int, verbose=True) -> Optional[int]:  
        """
            Read Moving Status(123).
            [Ex] gripper.read_moving_status(dxl_id=ysw.GRIPPER_ID_R)
            
            Return:
                int value, 0~255 
        """     
        val, dxl_comm_res, dxl_err = self.packet_hdlr.read1ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_MOVING_STATUS
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return None
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return None
        else:
            moving_status = val & 0xFF
            if verbose:
                logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Moving Status = {moving_status}, binary=0b{moving_status:08b}")
            return moving_status   
        
        

    def read_drive_mode(self, dxl_id: int, verbose=True) -> Optional[int]:  
        """
            Read Drive Mode(10).
        
            [Ex] gripper.read_drive_mode(dxl_id=ysw.GRIPPER_ID_R)
            
            Return:
                int value, 0~255
        """     
        val, dxl_comm_res, dxl_err = self.packet_hdlr.read1ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_DRIVE_MODE
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return None
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return None
        else:
            drive_mode = val & 0xFF
            if verbose:
                logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Drive Mode = {drive_mode}, binary=0b{drive_mode:08b}")
            return drive_mode    
        
        
    
    def read_profile_acceleration(self, dxl_id: int, verbose=True) -> Optional[int]:  
        """
            Read Profile Acceleration(108).
        
            [Ex] gripper.read_profile_acceleration(dxl_id=ysw.GRIPPER_ID_R)
            
            Return:
                int value, 0~32767
        """     
        val, dxl_comm_res, dxl_err = self.packet_hdlr.read4ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_PROFILE_ACCELRATION
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return None
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return None
        else:
            profile_acc = val 
            if verbose:
                logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Profile Acceleration = {profile_acc}")
            return profile_acc 
        
        
        
    def read_profile_velocity(self, dxl_id: int, verbose=True) -> Optional[int]:  
        """
            Read Profile Velocity(112).
        
            [Ex] gripper.read_profile_velocity(dxl_id=ysw.GRIPPER_ID_R)
            
            Return:
                int value, 0~32767
        """     
        val, dxl_comm_res, dxl_err = self.packet_hdlr.read4ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_PROFILE_VELOCITY
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return None
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return None
        else:
            profile_vel = val 
            if verbose:
                logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Profile Velocity = {profile_vel}")
            return profile_vel 
         
         
    #----------------------------------------------------------------------------
    # Register Write Functions
    # ---------------------------------------------------------------------------
    def write_torque_mode(self, dxl_id: int, torque_mode: int) -> bool:
        """
            Set Torque Mode 
            [Ex] gripper.write_torque_mode(dxl_id=dxl_id, torque_mode=ysw.DXL_TORQUE_ENABLE)
            
            Return:
                True: OK
                False: Error
        """    
        dxl_comm_res, dxl_err = self.packet_hdlr.write1ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_TORQUE_ENABLE,
            int(torque_mode)
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return False
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return False
        else:
            if (torque_mode == RI.DXL_TORQUE_DISABLE):      logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Torque is Disabled")
            elif (torque_mode == RI.DXL_TORQUE_ENABLE):     logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Torque is Enabled")
            return True
                
   
   
    def write_operation_mode(self, dxl_id: int, operation_mode: int) -> bool:
        """
            Set Operation Mode 
            [Ex] gripper.write_operation_mode(dxl_id=dxl_id, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
            
            Return:
                True: OK
                False: Error
        """  
        dxl_comm_res, dxl_err = self.packet_hdlr.write1ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_OPERATING_MODE,
            int(operation_mode)
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return False
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return False
        else:
            if (operation_mode == RI.DXL_CURRENT_CONTROL_MODE):                    
                logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set Current Control Mode")
            elif (operation_mode == RI.DXL_POSITION_CONTROL_MODE):                 
                logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set Postion Control Mode")
            elif (operation_mode == RI.DXL_EXTENDED_POSITION_CONTROL_MODE):   
                logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set Extended Position Control Mode")
            elif (operation_mode == RI.DXL_CURRENT_BASED_POSITION_CONTROL_MODE):   
                logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set Current-based Position Control Mode")
            else:   
                logging.error(f"Invalid DXL Operation Mode")  
                return False
            return True  
            


    def write_current_ampere(self, dxl_id: int, current_ampere: float) -> bool:
        """
            Apply Current value (unit = A)
            1-unit = 2.69mA 
            (+): Closed direction
            (-): Open direction
            [Ex] gripper.write_current_ampere(ysw.GRIPPER_ID_R, -0.02)
            
            Return:
                True: OK
                False: Error
        """      
        current_ampere_revised = current_ampere
        
        if (current_ampere > RI.DXL_CURRENT_LIMIT_MAX_A_RAINBOW):
            logging.info(f"[DXL ID:%d] Current MUST be less than {RI.DXL_CURRENT_LIMIT_MAX_A_RAINBOW}A")
            logging.info(f"[DXL ID:%d] Thus, Maximum Current({RI.DXL_CURRENT_LIMIT_MAX_A_RAINBOW}A) is applied")
            current_ampere_revised = RI.DXL_CURRENT_LIMIT_MAX_A_RAINBOW
            
        current_value = int(current_ampere_revised / RI.DXL_AMPERE_PER_UNIT)
        
        dxl_comm_res, dxl_err = self.packet_hdlr.write2ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_GOAL_CURRENT,
            current_value
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return False
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return False
        else:
            logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set Current [{current_ampere_revised} A]")
            return True



    def write_current_ampere_safety(self, dxl_id: int, current_ampere: float):        
        """
            Apply Current value (unit = A) [Ensuring connection version]
            1-unit = 2.69mA 
            (+): Closed direction
            (-): Open direction
            [Ex] gripper.write_current_ampere_safety(ysw.GRIPPER_ID_R, -0.02)
            
            Return: X
        """    
        
        current_ampere_revised = current_ampere
        
        if (current_ampere > RI.DXL_CURRENT_LIMIT_MAX_A_RAINBOW):
            logging.info(f"[DXL ID:%d] Current MUST be less than {RI.DXL_CURRENT_LIMIT_MAX_A_RAINBOW}A")
            logging.info(f"[DXL ID:%d] Thus, Maximum Current({RI.DXL_CURRENT_LIMIT_MAX_A_RAINBOW}A) is applied")
            current_ampere_revised = RI.DXL_CURRENT_LIMIT_MAX_A_RAINBOW
            
        current_value = int(current_ampere_revised / RI.DXL_AMPERE_PER_UNIT)
        
        while True:
            dxl_comm_res, dxl_err = self.packet_hdlr.write2ByteTxRx(
                self.port_hdlr,
                dxl_id,
                RI.DXL_ADDR_GOAL_CURRENT,
                current_value
            )
            
            if dxl_comm_res != RI.DXL_COMM_SUCCESS:
                logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            elif dxl_err != RI.DXL_NO_ERROR:
                logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            else:
                logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set Current [{current_ampere} A]")
                break
            
            time.sleep(0.1)
            print(f"[DXL ID]:{dxl_id} Retry...")
            


    def write_current_limit(self, dxl_id: int, current_limit_A: float) -> bool:        
        """
            Set Current Limit(Max) (unit = A)
            Raw value Range (0~1193)
            (Caution!): Default value is 1193(=3.2A) by ROBOTIS, But RAINBOW ROBOTICS limits it as 250(672.5mA)
            You NEVER use this function, because current limit setting is already done by RAINBOW ROBOTICS
            [Ex] gripper.write_current_limit(ysw.GRIPPER_ID_R, ysw.DXL_CURRENT_LIMIT_MAX_A_YSW)
            
            Return: 
                True: OK
                False: Error
        """  
        current_limit_raw = int(current_limit_A / RI.DXL_AMPERE_PER_UNIT)
        if (current_limit_raw > 250):     
            logging.info("Current Limint CANNOT be larger than 672.5mA")
            logging.info("Thus, Current limit is set to 672.5mA")
            current_limit_raw = 250

        dxl_comm_res, dxl_err = self.packet_hdlr.write2ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_CURRENT_LIMIT,
            current_limit_raw
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return False
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return False
        else:
            logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set Current Limit: {current_limit_A}A")
            return True
        
    
    
    def write_goalPosition_tick(self, dxl_id: int, goalPos_tick: int) -> bool:        
        """
            Write Goal Position using Raw Tick value. (without offset)
            This is Absolute Dynamixel Goal Position(tick) value.
            This is for "Current-based Position Control Mode" 
            [Ex] gripper.write_goalPosition_tick(ysw.GRIPPER_ID_R, 300)
            [Ex] gripper.write_goalPosition_tick(ysw.GRIPPER_ID_R, -200)
            
            Return: 
                True: OK
                False: Error
        """  
        goalPos_tick_signed = int(goalPos_tick)
        
        # Clamping Min/Max Values
        if (dxl_id == RI.GRIPPER_ID_R):
            if (goalPos_tick_signed > RI.GRIPPER_CLOSE_POS_RAWTICK_R):
                goalPos_tick_signed = RI.GRIPPER_CLOSE_POS_RAWTICK_R
            elif (goalPos_tick_signed < RI.GRIPPER_OPEN_POS_RAWTICK_R):
                goalPos_tick_signed = RI.GRIPPER_OPEN_POS_RAWTICK_R
        if (dxl_id == RI.GRIPPER_ID_L):
            if (goalPos_tick_signed > RI.GRIPPER_CLOSE_POS_RAWTICK_L):
                goalPos_tick_signed = RI.GRIPPER_CLOSE_POS_RAWTICK_L
            elif (goalPos_tick_signed < RI.GRIPPER_OPEN_POS_RAWTICK_L):
                goalPos_tick_signed = RI.GRIPPER_OPEN_POS_RAWTICK_L
        
        # Convert to uint32 
        goalPos_tick_uint32 = goalPos_tick_signed
        if goalPos_tick_uint32 < 0:
            goalPos_tick_uint32 += 2**32
                    
        dxl_comm_res, dxl_err = self.packet_hdlr.write4ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_GOAL_POSITION,
            int(goalPos_tick_uint32)
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return False
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return False
        else:
            logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set Goal Position [Tick]: " 
                         f"signed={goalPos_tick_signed}, uint32={goalPos_tick_uint32}")
            return True
        
        
    def write_PIDgain(self, dxl_id: int, Kp: int=RI.DXL_P_GAIN_POSCTRL, Ki: int=RI.DXL_I_GAIN_POSCTRL, Kd: int=RI.DXL_D_GAIN_POSCTRL) -> bool:        
        """
            Set P,I,D gain for DXL
            Raw value Range (0 ~ 16,383)
            [Ex] gripper.write_PIDgain(ysw.GRIPPER_ID_R, ysw.DXL_P_GAIN_POSCTRL, ysw.DXL_I_GAIN_POSCTRL, ysw.DXL_D_GAIN_POSCTRL)
            
            Return: 
                True: OK
                False: Error
        """  
        if (Kp < 0 or Ki < 0 or Kd < 0):     
            logging.error("Kp, Ki, Kd values cannot be less than 0")
            return False
        if (Kp > RI.DXL_PID_GAIN_LIMIT or Ki > RI.DXL_PID_GAIN_LIMIT or Kd > RI.DXL_PID_GAIN_LIMIT):     
            logging.error(f"Kp, Ki, Kd values cannot exceed limit [{RI.DXL_PID_GAIN_LIMIT}]")
            return False

        dxl_comm_res, dxl_err = self.packet_hdlr.write2ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_P_GAIN_POSCTRL,
            Kp
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return False
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return False
        else:
            logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set Kp: {Kp}")

        dxl_comm_res, dxl_err = self.packet_hdlr.write2ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_I_GAIN_POSCTRL,
            Ki
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return False
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return False
        else:
            logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set Ki: {Ki}")
  
        dxl_comm_res, dxl_err = self.packet_hdlr.write2ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_D_GAIN_POSCTRL,
            Kd
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return False
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return False
        else:
            logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set Kd: {Kd}")
            return True
        
        
    
    def write_drive_mode(self, dxl_id: int, register_8bit: int) -> bool:        
        """
            Set Drive Mode(10)

            Drive Mode is a 1-byte EEPROM register.
            Torque should be OFF before writing this value.
            
            register_8bit:
                Use binary literal, for example:
                    0b00000000  # velocity-based profile
                    0b00000100  # time-based profile
            
            [Ex] gripper.write_drive_mode(ysw.GRIPPER_ID_R, 0b00000100)
            
            Return: 
                True: OK
                False: Error
        """  
        drive_mode_value = int(register_8bit)
        
        if drive_mode_value < 0 or drive_mode_value > 255:
            logging.error(f"Drive Mode should be 1-byte value. Current value = {drive_mode_value}")
            return False
        
        dxl_comm_res, dxl_err = self.packet_hdlr.write1ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_DRIVE_MODE,
            drive_mode_value
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return False
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return False
        else:
            logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set Drive Mode: {format(register_8bit, '08b')}")
            return True
        
        
    
    def write_profile_acceleration(self, dxl_id: int, profile_acc_or_accTime: int) -> bool:        
        """
            Set Profile Acceleration(108)
            
            If Drive mode is Velocity-based profile,    then write "profile_acc"       -> unit: [rev/min^2]
            If Drive mode is Time-based profile,        then write "profile_accTime"   -> unit: [msec]      
            
            profile_acc_or_accTime: 4byte data (0~32767)
            
            [Ex] gripper.write_profile_acceleration(ysw.GRIPPER_ID_R, 1000)     # 1000[msec]
            
            Return: 
                True:  OK
                False: Error
        """  
        profile_acc_or_accTime = int(profile_acc_or_accTime)
        
        if profile_acc_or_accTime < 0 or profile_acc_or_accTime > 32767:
            logging.error(f"Profile Acceleration should be 4-byte value. Current value = {profile_acc_or_accTime}")
            return False
        
        dxl_comm_res, dxl_err = self.packet_hdlr.write4ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_PROFILE_ACCELRATION,
            profile_acc_or_accTime
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return False
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return False
        else:
            logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set Profile Acceleration(acc or accTime): {profile_acc_or_accTime}")
            return True
        
        
    
    def write_profile_velocity(self, dxl_id: int, profile_vel_or_totalTime: int) -> bool:        
        """
            Set Profile Velocity(112)
            
            If Drive mode is Velocity-based profile,    then write "profile_vel"         -> unit: [rev/min^2]
            If Drive mode is Time-based profile,        then write "profile_totalTime"   -> unit: [msec]      
            
            profile_vel_or_totalTime: 4byte data (0~32767)
            
            [Ex] gripper.write_profile_velocity(ysw.GRIPPER_ID_R, 2000)     # 2000[msec]
            
            Return: 
                True:  OK
                False: Error
        """  
        profile_vel_or_totalTime = int(profile_vel_or_totalTime)
        
        if profile_vel_or_totalTime < 0 or profile_vel_or_totalTime > 32767:
            logging.error(f"Profile Velocity should be 4-byte value. Current value = {profile_vel_or_totalTime}")
            return False
        
        dxl_comm_res, dxl_err = self.packet_hdlr.write4ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_PROFILE_VELOCITY,
            profile_vel_or_totalTime
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            return False
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            return False
        else:
            logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set Profile Velocity(vel or totalTime): {profile_vel_or_totalTime}")
            return True
        
        
        
    #----------------------------------------------------------------------------
    # Calculation Functions
    # ---------------------------------------------------------------------------
    def convert_pos_from_tick(self, pos_tick: float, unit=RI.RAD):
        if unit == RI.RAD:      pos = pos_tick / RI.DXL_RESOLUTION_TICKS_PER_REV * RI.TWO_PI
        elif unit == RI.DEG:    pos = pos_tick / RI.DXL_RESOLUTION_TICKS_PER_REV * RI.TWO_PI * RI.RAD2DEG
        return pos
            
            

    #----------------------------------------------------------------------------
    # Utility Functions (Mainly used)
    # ---------------------------------------------------------------------------
    def initialize_setting(
        self, 
        dxl_id=None, 
        operation_mode=RI.DXL_CURRENT_CONTROL_MODE, 
        drive_mode=RI.DXL_DRIVE_MODE_TIME_BASED_PROFILE,
        Kp=None,
        Ki=None, 
        Kd=None
    ) -> bool:
        """
            Initialize Gripper (Set Max Current + Set Operation Mode + Set PID gain for selected operation mode)
            This function should be used after "connect" function is used
            [Ex] gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
            
            If PID gain is set as "None", then the default values are used.
            
            Return:
                True: succesfully initialized
                False: not initialized
        """
        UTIL.print_start_title(f"Gripper [DXL ID:{dxl_id}] setting Starts")
        
        if ((dxl_id == None) or (dxl_id not in [RI.GRIPPER_ID_R, RI.GRIPPER_ID_L])):
            logging.error("Invalid DXL ID")
            return False
        
        if dxl_id in self.dxl_activated:
            # 1. Disable Torque mode
            while (self.write_torque_mode(dxl_id=dxl_id, torque_mode=RI.DXL_TORQUE_DISABLE) != True):
                logging.warning(f"[DXL ID:{dxl_id}] Retry torque disable")
                time.sleep(0.1)
            # self.write_current_limit(dxl_id=dxl_id, current_limit_A=RI.DXL_CURRENT_LIMIT_MAX_A_YSW)  # You Never use this (You don't need to set current limit)
            
            if operation_mode in RI.DXL_CONTROL_MODE:
                # 2. Set operation mode
                while (self.write_operation_mode(dxl_id=dxl_id, operation_mode=operation_mode) != True):
                    logging.warning(f"[DXL ID:{dxl_id}] Retry operation mode setting")
                    time.sleep(0.1)
                    
                # 3. Set PID gain for selected operation mode
                if (Kp is None and Ki is None and Kd is None):  
                    pass
                else:
                    while (self.tune_PIDgain(dxl_id=dxl_id, Kp=Kp, Ki=Ki, Kd=Kd) != True):
                        logging.warning(f"[DXL ID:{dxl_id}] Retry PID gain setting")
                        time.sleep(0.1)
            else:
                logging.error("Invalid DXL Operation Mode")
                return False
            
            # 4. Drive Mode - Profile
            if (drive_mode in (RI.DXL_DRIVE_MODE_VELOCITY_BASED_PROFILE, RI.DXL_DRIVE_MODE_TIME_BASED_PROFILE)):
                old_drive_mode = None
                
                while (old_drive_mode is None):
                    old_drive_mode = self.read_drive_mode(dxl_id, verbose=False)
                    
                    if old_drive_mode is None:
                        logging.warning(f"[DXL ID:{dxl_id}] Retry reading Drive Mode")
                        time.sleep(0.1)
                
                if drive_mode == RI.DXL_DRIVE_MODE_VELOCITY_BASED_PROFILE:
                    # bit 2 OFF
                    new_drive_mode = old_drive_mode & RI.DXL_DRIVE_MODE_VELOCITY_BASED_PROFILE_MASK
                elif drive_mode == RI.DXL_DRIVE_MODE_TIME_BASED_PROFILE:
                    # bit 2 ON
                    new_drive_mode = old_drive_mode | RI.DXL_DRIVE_MODE_TIME_BASED_PROFILE_MASK
                
                while (self.write_drive_mode(dxl_id, new_drive_mode) != True):
                    logging.warning(f"[DXL ID:{dxl_id}] Retry drive mode setting")
                    time.sleep(0.1)
                
            # 5. Enable Torque mode
            while (self.write_torque_mode(dxl_id=dxl_id, torque_mode=RI.DXL_TORQUE_ENABLE) != True):
                logging.warning(f"[DXL ID:{dxl_id}] Retry torque enable")
                time.sleep(0.1)
                
            UTIL.print_end_title(f"Gripper [DXL ID:{dxl_id}] setting is Done")
            return True
        else:
            logging.error(f"[DXL ID:{dxl_id}] is NOT activated")
            return False
        
        
        
    def disable(self, dxl_id=None) -> bool:
        """
            Disable Torque mode of Gripper (Free to move by external force)
            Essentially same as "write_torque_mode" function
            [Ex] gripper.disable(dxl_id=ysw.GRIPPER_ID_R)
            
            Return:
                True: succesfully disabled
                False: not disabled
        """
        UTIL.print_start_title(f"Gripper [DXL ID:{dxl_id}] Disable Starts")

        if (dxl_id is None) or (dxl_id not in [RI.GRIPPER_ID_R, RI.GRIPPER_ID_L]):
            logging.error("Invalid DXL ID")
            return False

        if dxl_id not in self.dxl_activated:
            logging.error(f"[DXL ID:{dxl_id}] is NOT activated")
            return False

        self.write_torque_mode(
            dxl_id=dxl_id,
            torque_mode=RI.DXL_TORQUE_DISABLE
        )

        UTIL.print_end_title(f"Gripper [DXL ID:{dxl_id}] Disable is Done")
        return True
    
    
    def enable(self, dxl_id=None) -> bool:
        """
            Enable Torque mode of Gripper (Free to move by external force)
            Essentially same as "write_torque_mode" function
            [Ex] gripper.enable(dxl_id=ysw.GRIPPER_ID_R)
            
            Return:
                True: succesfully enabled
                False: not enabled
        """
        UTIL.print_start_title(f"Gripper [DXL ID:{dxl_id}] Enable Starts")

        if (dxl_id is None) or (dxl_id not in [RI.GRIPPER_ID_R, RI.GRIPPER_ID_L]):
            logging.error("Invalid DXL ID")
            return False

        self.write_torque_mode(
            dxl_id=dxl_id,
            torque_mode=RI.DXL_TORQUE_ENABLE
        )

        UTIL.print_end_title(f"Gripper [DXL ID:{dxl_id}] Enable is Done")
        return True
    
    
    
    def homing(self, dxl_id=None, iter=100) -> bool:
        """
            Homing the DXL Angle Position Value
            After use this, Change the "robot_info.py" file 
                GRIPPER_OPEN_POS_RAWTICK_R
                GRIPPER_OPEN_POS_RAWTICK_L
                GRIPPER_CLOSE_POS_RAWTICK_R
                GRIPPER_CLOSE_POS_RAWTICK_L
                GRIPPER_POS_RANGE_TICK_R
                GRIPPER_POS_RANGE_TICK_L
                GRIPPER_POS_SCALE_TICK_TO_LINEAR_R
                GRIPPER_POS_SCALE_TICK_TO_LINEAR_L
            [Ex] gripper.homing(dxl_id=ysw.GRIPPER_ID_R, iter=50)
            
            Return:
                True: Homing success
                False: Error
        """
        UTIL.print_start_title(f"Gripper [DXL ID:{dxl_id}] Homing Starts")

        if (dxl_id is None) or (dxl_id not in [RI.GRIPPER_ID_R, RI.GRIPPER_ID_L]):
            logging.error("Invalid DXL ID")
            return False

        if dxl_id not in self.dxl_activated:
            logging.error(f"[DXL ID:{dxl_id}] is NOT activated")
            return False

        # 1. initialize_setting
        self.initialize_setting(dxl_id=dxl_id, operation_mode=RI.DXL_CURRENT_CONTROL_MODE)
        
        # 2. OPEN
        UTIL.print_start_subtitle(f"Gripper [DXL ID:{dxl_id}] OPEN Iteration Starts")
        posTick_open_list = []
        self.write_current_ampere(dxl_id=dxl_id, current_ampere=RI.GRIPPER_HOMING_OPEN_CURRENT)
        time.sleep(5)
        
        for i in range(iter):
            temp_posTick_open = self.read_position(dxl_id=dxl_id, unit=RI.TICK, homing_use=False, print_log=True)    # homing_use should be False!
            posTick_open_list.append(temp_posTick_open)
        
        if dxl_id == RI.GRIPPER_ID_R:     self.dxl_R_homing_openTick = int(UTIL.average_list(posTick_open_list))    
        elif dxl_id == RI.GRIPPER_ID_L:   self.dxl_L_homing_openTick = int(UTIL.average_list(posTick_open_list))
        UTIL.print_end_subtitle(f"Gripper [DXL ID:{dxl_id}] OPEN Iteration Ends")
        
        # 3. CLOSE
        UTIL.print_start_subtitle(f"Gripper [DXL ID:{dxl_id}] CLOSE Iteration Starts")
        posTick_close_list = []
        self.write_current_ampere(dxl_id=dxl_id, current_ampere=RI.GRIPPER_HOMING_CLOSE_CURRENT)
        time.sleep(5)
        
        for i in range(iter):
            temp_posTick_close = self.read_position(dxl_id=dxl_id, unit=RI.TICK, homing_use=False, print_log=True)   # homing_use should be False!
            posTick_close_list.append(temp_posTick_close)
        
        if dxl_id == RI.GRIPPER_ID_R:     self.dxl_R_homing_closeTick = int(UTIL.average_list(posTick_close_list))     
        elif dxl_id == RI.GRIPPER_ID_L:   self.dxl_L_homing_closeTick = int(UTIL.average_list(posTick_close_list))
        UTIL.print_end_subtitle(f"Gripper [DXL ID:{dxl_id}] CLOSE Iteration Ends")
        
        # 4. Disable Torque Mode
        self.disable(dxl_id=dxl_id)
        
        # 5. Calculate Angle Position Range
        if dxl_id == RI.GRIPPER_ID_R:     
            self.dxl_R_homing_rangeTick = int(self.dxl_R_homing_openTick - self.dxl_R_homing_closeTick)
            self.dxl_R_homing_LinearToTick = self.dxl_R_homing_rangeTick / RI.GRIPPER_POS_RANGE_LINEAR
        elif dxl_id == RI.GRIPPER_ID_L:   
            self.dxl_L_homing_rangeTick = int(self.dxl_L_homing_openTick - self.dxl_L_homing_closeTick)
            self.dxl_L_homing_LinearToTick = self.dxl_L_homing_rangeTick / RI.GRIPPER_POS_RANGE_LINEAR
        
        # 6. Print Homing Results
        if (dxl_id == RI.GRIPPER_ID_R):
            print(f"Gripper [DXL ID:{dxl_id}] OPEN Raw Position Tick: {self.dxl_R_homing_openTick} tick")
            print(f"Gripper [DXL ID:{dxl_id}] CLOSE Raw Position Tick: {self.dxl_R_homing_closeTick} tick")
            print(f"Gripper [DXL ID:{dxl_id}] Tick Range: {self.dxl_R_homing_rangeTick} tick")
            print("\n\n In robot_info.py file, Set the values:")
            print(f"GRIPPER_OPEN_POS_RAWTICK_R  = {self.dxl_R_homing_openTick}")
            print(f"GRIPPER_CLOSE_POS_RAWTICK_R = {self.dxl_R_homing_closeTick}")
            print(f"GRIPPER_POS_RANGE_TICK_R    = {self.dxl_R_homing_rangeTick}")
        elif (dxl_id == RI.GRIPPER_ID_L):
            print(f"Gripper [DXL ID:{dxl_id}] OPEN Raw Position Tick: {self.dxl_L_homing_openTick} tick")
            print(f"Gripper [DXL ID:{dxl_id}] CLOSE Raw Position Tick: {self.dxl_L_homing_closeTick} tick")
            print(f"Gripper [DXL ID:{dxl_id}] Tick Range: {self.dxl_L_homing_rangeTick} tick")
            print("\n\n In robot_info.py file, Set the values:")
            print(f"GRIPPER_OPEN_POS_RAWTICK_L  = {self.dxl_L_homing_openTick}")
            print(f"GRIPPER_CLOSE_POS_RAWTICK_L = {self.dxl_L_homing_closeTick}")
            print(f"GRIPPER_POS_RANGE_TICK_L    = {self.dxl_L_homing_rangeTick}")
        
        UTIL.print_end_title(f"Gripper [DXL ID:{dxl_id}] Homing is Done")
        return True
    
    
    
    def current_based_position_control_linear(
        self, 
        dxl_id: int, 
        current_ampere_mag: float=0.02, 
        linear_pos: float=0.0, 
        command_time_return=False,
        time_profile_used=False,
        profile_acceleration_ms: int = None,
        profile_velocity_ms: int = None,
    ):
        """
            Move gripper using linear position.
            MUST Use "Current-based Position Control Mode(5)" of Grippers
            Before use this function, operation mode should be "Current-based Position Control Mode(5)"
            Unit: [A], [mm]
            
            current_ampere_mag:
                Absolute value of Max Limit of current (default: 20mA)
                It must not exceed 0.6725 A
            
            linear_pos:
                0.0  -> closed
                91.0 -> open
                
            command_time_return:
                If it is True, save the "command_time" to "self.command_time"
                
            time_profile_used:
                If it is True, use Drive Mode(10) - Time-based Profile 
                It uses Profile Acceleration(108) and Profile Velocity(102) with time[msec] unit
                Make sure that the Drive Mode(10) is set as "Time-based Profile" mode (Bit index 2 should be 1) 
                
            [Ex]
                gripper.current_based_position_control_linear(ysw.GRIPPER_ID_R, 0.02, 0.0)   # close
                gripper.current_based_position_control_linear(ysw.GRIPPER_ID_R, 0.02, 91.0)  # open
                gripper.current_based_position_control_linear(ysw.GRIPPER_ID_R, 0.02, GRIPPER_POS_RANGE_LINEAR)  # Open
        """
        UTIL.print_start_title(f"Gripper [DXL ID:{dxl_id}] Current-based Position Control Starts")
        print(f"Desired Linear Position = {linear_pos}mm")
        
        if (dxl_id is None) or (dxl_id not in [RI.GRIPPER_ID_R, RI.GRIPPER_ID_L]):
            logging.error("Invalid DXL ID")
            return False

        if dxl_id not in self.dxl_activated:
            logging.error(f"[DXL ID:{dxl_id}] is NOT activated")
            return False
        
        mode = self.read_operation_mode(dxl_id)
        if mode != RI.DXL_CURRENT_BASED_POSITION_CONTROL_MODE:
            logging.error(f"[DXL ID:{dxl_id}] is NOT set as Current-based position control mode"
                          f"Current operation mode = {mode}"
            )
            return False
        
        # 1. Clamp desired linear position: 0mm(Closed) to maximum value(=91mm, Open)
        if (linear_pos < 0):
            print("Position cannot be less than 0[mm]")
            print("Thus, Min value(CLOSED) is applied: 0[mm]")
        elif (linear_pos > RI.GRIPPER_POS_RANGE_LINEAR):
            print(f"Position cannot be larger than {RI.GRIPPER_POS_RANGE_LINEAR}[mm]")
            print(f"Thus, Max value(OPEN) is applied: {RI.GRIPPER_POS_RANGE_LINEAR}[mm]")
        desired_pos_linear = max(0.0, min(linear_pos, RI.GRIPPER_POS_RANGE_LINEAR))     
        
        # 2. Calculate the Raw Tick value for desired linear position
        if dxl_id == RI.GRIPPER_ID_R:
            desired_pos_tick = desired_pos_linear * RI.GRIPPER_POS_SCALE_LINEAR_TO_TICK_R
            offset_tick = RI.GRIPPER_CLOSE_POS_RAWTICK_R
        elif dxl_id == RI.GRIPPER_ID_L:
            desired_pos_tick = desired_pos_linear * RI.GRIPPER_POS_SCALE_LINEAR_TO_TICK_L
            offset_tick = RI.GRIPPER_CLOSE_POS_RAWTICK_L      
        desired_pos_tick = int(desired_pos_tick + offset_tick)
        
        # 3. Set Goal Current (Should be less than 0.6725A)
        current_A_abs = abs(current_ampere_mag)
        current_A_abs = max(0.0, min(current_A_abs, RI.DXL_CURRENT_LIMIT_MAX_A_YSW)) 
        self.write_current_ampere(dxl_id, current_A_abs)
        
        # 4. (optional) Set Time-based Profile
        # In Time-based Profile:
        #   Profile Acceleration(108) = acceleration time [ms]
        #   Profile Velocity(112)     = total profile time [ms]
        if time_profile_used:
            if profile_acceleration_ms is not None:
                profile_acceleration_ms = int(profile_acceleration_ms)

                if profile_acceleration_ms < 0:
                    logging.error("Profile Acceleration should be non-negative.")
                    return False

                if not self.write_profile_acceleration(dxl_id, profile_acceleration_ms):
                    logging.error("Failed to write Profile Acceleration.")
                    return False

            if profile_velocity_ms is not None:
                profile_velocity_ms = int(profile_velocity_ms)

                if profile_velocity_ms < 0:
                    logging.error("Profile Velocity should be non-negative.")
                    return False

                if not self.write_profile_velocity(dxl_id, profile_velocity_ms):
                    logging.error("Failed to write Profile Velocity.")
                    return False
        
        # 4. Set Goal Position
        if command_time_return:
            self.command_time = time.perf_counter()
        if (self.write_goalPosition_tick(dxl_id=dxl_id, goalPos_tick=desired_pos_tick)):
            UTIL.print_end_title(f"Gripper [DXL ID:{dxl_id}] Current-based Position Control Ends")
            return True
        return False 



    def extended_position_control_linear(self, dxl_id: int, linear_pos: float=0.0) -> bool:
        """
            Move gripper using linear position.
            MUST Use "Extended Position Control Mode(4)" of Grippers
            Before use this function, operation mode should be "Extended Position Control Mode(4)"
            Unit: [mm]

            linear_pos:
                0.0  -> closed
                91.0 -> open

            [Ex]
                gripper.extended_position_control_linear(ysw.GRIPPER_ID_R, 0.0)   # close
                gripper.extended_position_control_linear(ysw.GRIPPER_ID_R, 91.0)  # open
                gripper.extended_position_control_linear(ysw.GRIPPER_ID_R, GRIPPER_POS_RANGE_LINEAR)  # open
        """
        UTIL.print_start_title(f"Gripper [DXL ID:{dxl_id}] Extended Position Control Starts")
        print(f"Desired Linear Position = {linear_pos}mm")

        if (dxl_id is None) or (dxl_id not in [RI.GRIPPER_ID_R, RI.GRIPPER_ID_L]):
            logging.error("Invalid DXL ID")
            return False

        if dxl_id not in self.dxl_activated:
            logging.error(f"[DXL ID:{dxl_id}] is NOT activated")
            return False

        mode = self.read_operation_mode(dxl_id)
        if mode != RI.DXL_EXTENDED_POSITION_CONTROL_MODE:
            logging.error(
                f"[DXL ID:{dxl_id}] is NOT set as Extended Position Control Mode. "
                f"Current operation mode = {mode}"
            )
            return False

        # 1. Clamp desired linear position: 0mm(Closed) to maximum value(=91mm, Open)
        if (linear_pos < 0):
            print("Position cannot be less than 0[mm]")
            print("Thus, Min value(CLOSED) is applied: 0[mm]")
        elif (linear_pos > RI.GRIPPER_POS_RANGE_LINEAR):
            print(f"Position cannot be larger than {RI.GRIPPER_POS_RANGE_LINEAR}[mm]")
            print(f"Thus, Max value(OPEN) is applied: {RI.GRIPPER_POS_RANGE_LINEAR}[mm]")

        desired_pos_linear = max(0.0, min(linear_pos, RI.GRIPPER_POS_RANGE_LINEAR))

        # 2. Calculate the Raw Tick value for desired linear position
        if dxl_id == RI.GRIPPER_ID_R:
            desired_pos_tick = desired_pos_linear * RI.GRIPPER_POS_SCALE_LINEAR_TO_TICK_R
            offset_tick = RI.GRIPPER_CLOSE_POS_RAWTICK_R

        elif dxl_id == RI.GRIPPER_ID_L:
            desired_pos_tick = desired_pos_linear * RI.GRIPPER_POS_SCALE_LINEAR_TO_TICK_L
            offset_tick = RI.GRIPPER_CLOSE_POS_RAWTICK_L

        desired_pos_tick = int(desired_pos_tick + offset_tick)

        # 3. Set Goal Position
        if self.write_goalPosition_tick(dxl_id=dxl_id, goalPos_tick=desired_pos_tick):
            UTIL.print_end_title(f"Gripper [DXL ID:{dxl_id}] Extended Position Control Ends")
            return True

        return False



    def tune_PIDgain(self, dxl_id: int, Kp: int=RI.DXL_P_GAIN_POSCTRL, Ki: int=RI.DXL_I_GAIN_POSCTRL, Kd: int=RI.DXL_D_GAIN_POSCTRL) -> bool:
        """
            Setting the PID gain of Present Control(operation) mode of Gripper
            
            Before changing the PID gains, make sure the desired operation mode is selected first.
            After use this function, the setting is maintained.
            
            PID gain은 "write_operation_mode" 함수 실행으로 operation mode가 변경될 때마다 초기화값으로 초기화됨. 
            따라서, operation mode를 바꾸게 되면 다시 실행해야 함.
                
            [Ex] If you want to change the PID gain of "current-based position control mode",
                gripper.initialize_setting(dxl_id=dxl_id, operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE)       # Necessary
                gripper.tune_PIDgain(ysw.GRIPPER_ID_R, 800, 10, 0)   # close
        """
        UTIL.print_start_title(f"Gripper [DXL ID:{dxl_id}] Tuning PID gain Starts")
        
        if (dxl_id is None) or (dxl_id not in [RI.GRIPPER_ID_R, RI.GRIPPER_ID_L]):
            logging.error("Invalid DXL ID")
            return False

        if dxl_id not in self.dxl_activated:
            logging.error(f"[DXL ID:{dxl_id}] is NOT activated")
            return False
        
        [Kp_old, Ki_old, Kd_old] = self.read_PIDgain(dxl_id)
        logging.info(f"[DXL ID:{dxl_id}] Original PID gain: Kp={Kp_old}, Ki={Ki_old}, Kd={Kd_old}")
        
        self.write_PIDgain(dxl_id, Kp, Ki, Kd)
        
        [Kp_new, Ki_new, Kd_new] = self.read_PIDgain(dxl_id)
        logging.info(f"[DXL ID:{dxl_id}] Newly tuned PID gain: Kp={Kp_new}, Ki={Ki_new}, Kd={Kd_new}")
         
        UTIL.print_end_title(f"Gripper [DXL ID:{dxl_id}] Tuning PID gain Ends")
        return True




# =============================================================================
# General Functions for Gripper
# =============================================================================
def sign(x):
    if x > 0.0:       return 1.0
    elif x < 0.0:     return -1.0
    else:           return 0.0
    

def trapezoidal_position_control(
    gripper,
    dxl_id,
    target_pos_mm,
    current_ampere_mag=0.02,
    v_max=40.0,             # [mm/s]
    a_max=50.0,             # [mm/s^2]
    dt=0.01,                # [sec], 100Hz
    trajectory_time=3.0,    # [sec], total profile time
    plot=True,
):
    """
        Time-based trapezoidal position control.
        
        Current actual position is read once at the beginning.
        Then a trapezoidal/triangular velocity profile is generated so that

            start_pos_mm -> target_pos_mm

        is completed exactly in trajectory_time seconds.

        Constraints:
            |v| <= v_max
            |a| <= a_max

        If the requested trajectory_time is too short to satisfy these limits,
        the function returns False.

        Return:
            True
                Trajectory command completed successfully.

            False
                Invalid/infeasible trajectory or communication failure.
    """
    
    def plot_trapezoidal_profile():
        if not plot or len(t_cmd_list) == 0:
            return

        t_cmd = np.array(t_cmd_list)
        x_cmd = np.array(x_cmd_list)
        v_cmd = np.array(v_cmd_list)
        a_cmd = np.array(a_cmd_list)

        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

        # Commanded profile
        axes[0].plot(t_cmd, x_cmd, label="commanded position")
        axes[1].plot(t_cmd, v_cmd, label="commanded velocity")
        axes[2].plot(t_cmd, a_cmd, label="commanded acceleration")

        # Reference lines
        axes[0].axhline(target_pos_mm, linestyle="--", label="target position")

        axes[1].axhline(v_max, linestyle="--", label="+v_max")
        axes[1].axhline(-v_max, linestyle="--", label="-v_max")

        axes[2].axhline(a_max, linestyle="--", label="+a_max")
        axes[2].axhline(-a_max, linestyle="--", label="-a_max")

        # Labels
        axes[0].set_ylabel("Position [mm]")
        axes[1].set_ylabel("Velocity [mm/s]")
        axes[2].set_ylabel("Acceleration [mm/s²]")
        axes[2].set_xlabel("Time [s]")

        for ax in axes:
            ax.grid(True)
            ax.legend()

        fig.suptitle("Gripper Trapezoidal Position Control Profile")
        plt.tight_layout()
        plt.show()
        
        
    # ============================================================
    # 1. Check parameters
    # ============================================================

    target_pos_mm = float(target_pos_mm)
    trajectory_time = float(trajectory_time)

    v_max = abs(float(v_max))
    a_max = abs(float(a_max))
    dt = float(dt)

    if trajectory_time <= 0.0:
        raise ValueError("trajectory_time must be positive.")
    
    if v_max <= 0.0:
        raise ValueError("v_max must be positive.")

    if a_max <= 0.0:
        raise ValueError("a_max must be positive.")

    if dt <= 0.0:
        raise ValueError("dt must be positive.")
        
        
    # ============================================================
    # 2. Read ACTUAL start position
    # ============================================================
    start_pos_mm = gripper.read_position(
        dxl_id=dxl_id,
        unit=RI.MM,
        homing_use=True,
        verbose=False,
    )

    if start_pos_mm is None:
        logging.error(
            f"Failed to read gripper position. "
            f"dxl_id={dxl_id}"
        )
        return False

    start_pos_mm = float(start_pos_mm)

    displacement = target_pos_mm - start_pos_mm
    distance = abs(displacement)

    # direction:
    # +1 : opening direction
    # -1 : closing direction
    if displacement > 0.0:
        direction = 1.0
    elif displacement < 0.0:
        direction = -1.0
    else:
        direction = 0.0

    print(
        f"[Gripper Trapezoidal Profile]\n"
        f"  start            = {start_pos_mm:.3f} mm\n"
        f"  target           = {target_pos_mm:.3f} mm\n"
        f"  distance         = {distance:.3f} mm\n"
        f"  trajectory_time  = {trajectory_time:.3f} sec"
    )        

        
    # ============================================================
    # 3. Already at target
    # ============================================================
    if distance < 1e-9:
        rv = gripper.current_based_position_control_linear(
            dxl_id,
            current_ampere_mag=current_ampere_mag,
            linear_pos=target_pos_mm,
        )
        return rv    
    

    # ============================================================
    # 4. Generate profile parameters
    # ============================================================
    #
    # Symmetric trajectory:
    #
    #   acceleration      constant velocity      deceleration
    #
    #        /--------\
    #       /          \
    # -----/            \-----
    #
    #
    # t_acc   : acceleration time
    # t_const : constant velocity time
    # t_acc   : deceleration time
    #
    # trajectory_time = 2*t_acc + t_const
    #
    # distance = v_peak * (trajectory_time - t_acc)
    #
    # ============================================================

    # ------------------------------------------------------------
    # First try a triangular profile
    # ------------------------------------------------------------
    #
    #       /\
    #      /  \
    # ----    ----
    #
    # t_acc = T/2
    #
    # distance = 1/2 * T * v_peak
    #
    # therefore
    #
    # v_peak = 2*distance / T
    #
    # ------------------------------------------------------------

    triangular_peak_velocity = (2.0 * distance / trajectory_time)    
    
    # [Case 1]: triangular profile is possible within v_max
    if triangular_peak_velocity <= v_max:
        t_acc = trajectory_time / 2.0
        t_const = 0.0

        v_peak = triangular_peak_velocity
        a_profile = v_peak / t_acc
        
        profile_type = "Triangular"
        
    # [Case 2]: v_max is reached -> trapezoidal profile
    else:
        v_peak = v_max

        # distance = v_peak * (T - t_acc)
        # therefore, t_acc = T - distance / v_peak

        t_acc = (trajectory_time - distance / v_peak)

        if t_acc <= 0.0:
            logging.error("Requested trajectory_time is too short for the given v_max.")
            return False

        t_const = (trajectory_time - 2.0 * t_acc)

        if t_const < -1e-9:
            logging.error("Invalid trapezoidal profile.")
            return False

        # numerical protection
        t_const = max(0.0, t_const)
        a_profile = v_peak / t_acc
        
        profile_type = "Trapezoidal"


    # ============================================================
    # 5. Check acceleration constraint
    # ============================================================
    if a_profile > a_max + 1e-9:
        logging.error(
            f"Requested move_time   = {trajectory_time:.3f} sec is too short.\n"
            f"Required acceleration = {a_profile:.3f} mm/s²\n"
            f"a_max                 = {a_max:.3f} mm/s²"
        )
        return False

    # Distances of each section
    d_acc   = (0.5 * a_profile * t_acc**2)
    d_const = (v_peak * t_const)

    print(
        f"  profile      = {profile_type}\n"
        f"  t_acc        = {t_acc:.3f} sec\n"
        f"  t_const      = {t_const:.3f} sec\n"
        f"  t_dec        = {t_acc:.3f} sec\n"
        f"  peak velocity= {v_peak:.3f} mm/s\n"
        f"  acceleration = {a_profile:.3f} mm/s²"
    )


    # ============================================================
    # 6. Data for plotting
    # ============================================================
    t_cmd_list = []
    x_cmd_list = []
    v_cmd_list = []
    a_cmd_list = []

    # ============================================================
    # 7. Trajectory generator function
    # ============================================================
    def calculate_command(t):
        # --------------------------------------------------------
        # Phase 1: acceleration
        # --------------------------------------------------------
        if t < t_acc:
            s = (0.5 * a_profile* t**2)
            v = (a_profile * t)
            a = a_profile

        # --------------------------------------------------------
        # Phase 2: constant velocity
        # --------------------------------------------------------
        elif t < t_acc + t_const:
            t_local = (t - t_acc)
            s = (d_acc + v_peak * t_local)
            v = v_peak
            a = 0.0

        # --------------------------------------------------------
        # Phase 3: deceleration
        # --------------------------------------------------------
        elif t < trajectory_time:
            t_local = (t - t_acc - t_const)
            s = (
                d_acc 
                + d_const
                + v_peak * t_local
                - 0.5 * a_profile * t_local**2
            )
            v = (v_peak - a_profile * t_local)
            a = -a_profile

        # --------------------------------------------------------
        # Final point
        # --------------------------------------------------------
        else:
            s = distance
            v = 0.0
            a = 0.0

        x = (start_pos_mm + direction * s)
        v = direction * v
        a = direction * a

        return x, v, a


    # ============================================================
    # 8. Execute trajectory
    # ============================================================
    trajectory_start_time = time.perf_counter()

    # Desired sample times
    t_samples = np.arange(0.0, trajectory_time, dt)

    # Ensure exact final point t = trajectory_time
    t_samples = np.append(t_samples, trajectory_time)
    
    # Timing check
    command_start_time_list = []
    command_duration_list = []
    lateness_list = []

    previous_command_start_time = None
    actual_dt_list = []

    for t_cmd in t_samples:
        # Wait until scheduled command time
        target_time = (trajectory_start_time + t_cmd)
        sleep_time  = (target_time - time.perf_counter())

        if sleep_time > 0.0:
            time.sleep(sleep_time)
            
        # Actual command start time
        command_start_time = time.perf_counter()

        # 예정 시간보다 얼마나 늦었는지
        lateness = command_start_time - target_time

        command_start_time_list.append(command_start_time - trajectory_start_time)
        lateness_list.append(lateness)

        # 실제 command-to-command interval
        if previous_command_start_time is not None:
            actual_dt = command_start_time - previous_command_start_time
            actual_dt_list.append(actual_dt)

        previous_command_start_time = command_start_time    

        # Generate commanded state
        x_cmd, v_cmd, a_cmd = calculate_command(t_cmd)

        # Save commanded trajectory
        t_cmd_list.append(t_cmd)
        x_cmd_list.append(x_cmd)
        v_cmd_list.append(v_cmd)
        a_cmd_list.append(a_cmd)

        # Send position reference
        send_start_time = time.perf_counter()
        
        rv = gripper.current_based_position_control_linear(
            dxl_id,
            current_ampere_mag=current_ampere_mag,
            linear_pos=x_cmd,
        )
        
        send_end_time = time.perf_counter()
        
        command_duration = send_end_time - send_start_time
        command_duration_list.append(command_duration)

        if rv is False:
            logging.error("Failed to send gripper position command")
            plot_trapezoidal_profile()
            return False
        
        
    # ============================================================
    # Timing analysis
    # ============================================================
    if len(actual_dt_list) > 0:
        actual_dt_arr = np.array(actual_dt_list)
        command_duration_arr = np.array(command_duration_list)
        lateness_arr = np.array(lateness_list)

        actual_dt_mean = np.mean(actual_dt_arr)
        actual_dt_std = np.std(actual_dt_arr)
        actual_dt_max = np.max(actual_dt_arr)
        actual_dt_min = np.min(actual_dt_arr)

        command_duration_mean = np.mean(command_duration_arr)
        command_duration_max = np.max(command_duration_arr)

        lateness_max = np.max(lateness_arr)

        # dt보다 실제 command 실행 시간이 긴 경우
        command_overrun_count = np.sum(command_duration_arr > dt)

        # 실제 command interval이 dt보다 10% 이상 긴 경우
        deadline_miss_count = np.sum(actual_dt_arr > 1.1 * dt)

        print(
            "\n[Trajectory Timing Check]\n"
            f"  requested dt          = {dt*1000:.3f} ms\n"
            f"  requested frequency   = {1.0/dt:.2f} Hz\n"
            f"\n"
            f"  actual dt mean        = {actual_dt_mean*1000:.3f} ms\n"
            f"  actual dt std         = {actual_dt_std*1000:.3f} ms\n"
            f"  actual dt min         = {actual_dt_min*1000:.3f} ms\n"
            f"  actual dt max         = {actual_dt_max*1000:.3f} ms\n"
            f"\n"
            f"  command duration mean = {command_duration_mean*1000:.3f} ms\n"
            f"  command duration max  = {command_duration_max*1000:.3f} ms\n"
            f"\n"
            f"  max lateness          = {lateness_max*1000:.3f} ms\n"
            f"  command overruns      = {command_overrun_count}\n"
            f"  deadline misses       = {deadline_miss_count}"
        )

        # --------------------------------------------------------
        # dt validity check
        # --------------------------------------------------------
        if command_duration_max >= dt:
            logging.warning(
                f"dt={dt*1000:.3f} ms is TOO SHORT. "
                f"Maximum command execution time was "
                f"{command_duration_max*1000:.3f} ms."
            )
        elif command_duration_max >= 0.8 * dt:
            logging.warning(
                f"dt={dt*1000:.3f} ms has little timing margin. "
                f"Maximum command execution time was "
                f"{command_duration_max*1000:.3f} ms."
            )
        elif deadline_miss_count > 0:
            logging.warning(
                f"Timing jitter detected: "
                f"{deadline_miss_count} command intervals "
                f"exceeded 110% of requested dt."
            )
        else:
            print(
                f"[Timing OK] dt={dt*1000:.3f} ms "
                f"({1.0/dt:.1f} Hz) appears feasible."
            )


    # ============================================================
    # 9. Read actual final position
    # ============================================================
    actual_pos_mm = gripper.read_position(
        dxl_id=dxl_id,
        unit=RI.MM,
        homing_use=True,
        verbose=False,
    )

    if actual_pos_mm is not None:
        pos_error = (target_pos_mm - actual_pos_mm)

        print(
            f"[Gripper trapezoid position control is Done]\n"
            f"  target         = {target_pos_mm:.3f} mm\n"
            f"  actual         = {actual_pos_mm:.3f} mm\n"
            f"  position error = {pos_error:.3f} mm"
        )

    else:
        logging.warning("Failed to read final actual position")

    # ============================================================
    # 10. Plot
    # ============================================================
    plot_trapezoidal_profile()

    return True    
    


def monitor_gripper_current(
    gripper,
    dxl_id,
    duration_sec=None,
    sample_dt=0.01,       # 100 Hz
    title="Gripper Current Monitoring",
    verbose=True,
):
    """
    Dynamixel gripper current를 모니터링하고, 측정이 끝난 뒤 plot.
    duration_sec 시간이 지나거나 Ctrl+C로 멈추면 마지막에 그래프를 그림.

    Args:
        gripper      : Gripper object
        dxl_id       : Dynamixel ID
        duration_sec : 측정 시간 [sec]. None이면 Ctrl+C까지 계속 측정
        sample_dt    : sampling interval [sec]
        plot_abs     : True이면 abs(current)도 같이 표시
        title        : plot title
        verbose      : If it is True, print the current values

    Return:
        result = {
            "time": time_list,
            "current": current_list,
        }
    """

    time_list = []
    current_list = []

    start_time = time.time()

    logging.info("Start current monitoring.")
    logging.info("Press Ctrl+C to stop.")

    try:
        while True:
            now = time.time()
            elapsed = now - start_time

            if duration_sec is not None and elapsed >= duration_sec:
                break

            current_A = gripper.read_current_ampere(
                dxl_id=dxl_id,
                verbose=verbose,
            )

            if current_A is not None:
                time_list.append(elapsed)
                current_list.append(current_A)

            time.sleep(sample_dt)

    except KeyboardInterrupt:
        print("\n[INFO] Current monitoring stopped by user.")

    result = {
        "time": np.array(time_list),
        "current": np.array(current_list),
    }

    logging.info("Current monitoring finished.")
    logging.info(f"Num samples: {len(time_list)}")

    if len(current_list) == 0:
        print("[WARNING] No current data to plot.")
        return result
    
    current_array = np.array(current_list)
    time_array = np.array(time_list)

    print(f"Current mean     : {np.mean(current_array):.4f} A")
    print(f"Current max      : {np.max(current_array):.4f} A")
    print(f"Current min      : {np.min(current_array):.4f} A")
    print(f"Abs current max  : {np.max(np.abs(current_array)):.4f} A")

    # -------------------------------------------------
    # Plot after monitoring finished
    # -------------------------------------------------
    plt.figure(figsize=(10, 4))
    plt.plot(time_array, current_array, label="current [A]")

    plt.xlabel("Time [s]")
    plt.ylabel("Current [A]")
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.show()

    return result


def monitor_gripper_live(
    gripper,
    dxl_id,
    signal="current",     # "current" or "position"
    duration_sec=None,
    sample_dt=0.01,       # [sec]
    title="Gripper Monitoring",
    verbose=False,
    ylim=None,            # 예: (-0.1, 0.1)
    window_sec=None,      # None이면 전체 시간 표시, 예: 10.0이면 최근 10초만 표시
):
    """
    Dynamixel gripper data current/position 를 실시간으로 모니터링하고 plot.

    권장 사용:
        %matplotlib tk

    Args:
        gripper      : Gripper object
        dxl_id       : Dynamixel ID
        signal            : "current" 또는 "position"
        duration_sec : 측정 시간 [sec]. None이면 Ctrl+C까지 계속 측정
        sample_dt    : sampling interval [sec]
        title        : plot title
        verbose      : True이면 read_current_ampere 로그 출력
        ylim         : y축 고정 범위. 예: (-0.1, 0.1)
        window_sec   : 최근 window_sec초만 표시. None이면 전체 시간 표시

    Return:
        result = {
            "time": np.array(time_list),
            "value": np.array(value_list),
            "signal": signal,
            "unit": unit,
        }
    """
    
    # -------------------------------------------------
    # Signal setting
    # -------------------------------------------------
    signal = signal.lower()
    
    if signal == "current":
        ylabel = "Current [A]"
        line_label = "current [A]"
        unit = "A"

        if title is None:
            title = "Gripper Current Monitoring"

        def read_value():
            return gripper.read_current_ampere(
                dxl_id=dxl_id,
                verbose=verbose,
            )

    elif signal == "position":
        ylabel = "Position [mm]"
        line_label = "position [mm]"
        unit = "mm"

        if title is None:
            title = "Gripper Position Monitoring"

        def read_value():
            return gripper.read_position(
                dxl_id=dxl_id,
                unit=RI.MM,
                homing_use=True,
                verbose=verbose,
            )
            
    else:
        raise ValueError('Invalid signal name')

    # -------------------------------------------------
    # Data
    # -------------------------------------------------
    time_list = []
    value_list = []

    # 이전 figure 정리
    plt.close("all")

    # Interactive mode ON
    plt.ion()

    fig, ax = plt.subplots(figsize=(20, 12), dpi=180)

    line_value, = ax.plot([], [], label=line_label, linewidth=3.0)

    ax.set_xlabel("Time [s]", fontsize=16)
    ax.set_ylabel(ylabel, fontsize=16)
    ax.set_title(title, fontsize=20)
    
    ax.tick_params(axis="both", labelsize=16)
    ax.grid(True)
    ax.legend(fontsize=16)

    if ylim is not None:
        ax.set_ylim(ylim[0], ylim[1])

    plt.tight_layout()
    plt.show(block=False)
    
    try:
        manager = plt.get_current_fig_manager()
        manager.window.wm_geometry("1200x700+100+100")
    except Exception:
        pass

    # -------------------------------------------------
    # Monitoring loop
    # -------------------------------------------------
    start_time = time.perf_counter()
    next_sample_time = start_time

    logging.info("Start current monitoring.")
    logging.info("Press Ctrl+C to stop.")

    try:
        while True:
            now = time.perf_counter()
            elapsed = now - start_time

            if duration_sec is not None and elapsed >= duration_sec:
                break

            # -------------------------------------------------
            # Read selected signal
            # -------------------------------------------------
            value = read_value()
            
            if value is not None:
                time_list.append(elapsed)
                value_list.append(value)
                
                t_arr = np.array(time_list)
                y_arr = np.array(value_list)

                # -------------------------------------------------
                # Update plot data
                # -------------------------------------------------
                line_value.set_data(t_arr, y_arr)

                # -------------------------------------------------
                # X-axis update
                # -------------------------------------------------
                if window_sec is None:
                    x_min = 0.0
                    x_max = max(1.0, elapsed)
                else:
                    x_min = max(0.0, elapsed - window_sec)
                    x_max = max(window_sec, elapsed)

                ax.set_xlim(x_min, x_max)
                
                # -------------------------------------------------
                # Y-axis update
                # -------------------------------------------------
                if ylim is None:
                    if window_sec is None:
                        y_view = y_arr
                    else:
                        mask = t_arr >= x_min
                        y_view = y_arr[mask]

                    if len(y_view) > 0:
                        y_min = float(np.min(y_view))
                        y_max = float(np.max(y_view))

                        # current는 0 기준을 같이 보여주는 게 보기 좋음
                        if signal == "current":
                            y_min = min(y_min, 0.0)
                            y_max = max(y_max, 0.0)

                        if abs(y_max - y_min) < 1e-6:
                            margin = 0.01 if signal == "current" else 1.0
                        else:
                            margin = 0.2 * abs(y_max - y_min)

                        ax.set_ylim(y_min - margin, y_max + margin)

                # -------------------------------------------------
                # Draw
                # -------------------------------------------------
                fig.canvas.draw()
                fig.canvas.flush_events()
                plt.pause(0.001)

            # -------------------------------------------------
            # Sampling timing
            # -------------------------------------------------
            next_sample_time += sample_dt
            sleep_time = next_sample_time - time.perf_counter()

            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                # 통신/plot이 sample_dt보다 오래 걸리면 기준시간 재설정
                next_sample_time = time.perf_counter()

    except KeyboardInterrupt:
        print("\n[INFO] Current monitoring stopped by user.")

    finally:
        plt.ioff()

    # -------------------------------------------------
    # Result
    # -------------------------------------------------
    result = {
        "time": np.array(time_list),
        "value": np.array(value_list),
        "signal": signal,
        "unit": unit,
    }

    logging.info(f"Gripper {signal} monitoring finished.")
    logging.info(f"Num samples: {len(time_list)}")

    if len(value_list) > 0:
        value_array = np.array(value_list)

        print(f"{signal.capitalize()} mean     : {np.mean(value_array):.4f} {unit}")
        print(f"{signal.capitalize()} max      : {np.max(value_array):.4f} {unit}")
        print(f"{signal.capitalize()} min      : {np.min(value_array):.4f} {unit}")

    return result



# Check the Step Response of Gripper Position[mm] 
def record_response_for_stepPositionReference(
    gripper,
    dxl_id,
    target_pos_mm,
    current_ampere_mag=0.02,
    pre_record_sec=0.5,     # 명령 전 몇 초 동안 기록할지
    sample_dt=0.01,         # position read period [sec]
    settled_time=3.0,       # 1% error band 안에 이 시간만큼 머물면 완료
    timeout_sec=5.0,        # 명령 후 최대 대기 시간
    title="Gripper Position Step Response",
    plot=True,
):
    """
    Gripper position command를 보내기 전부터 read_position으로 데이터를 모으고,
    step position reference에 대한 actual position response를 기록한 뒤 plot.

    Ensure that this function is used in operation mode 5:
        Current-based Position Control Mode

    Step response metrics:
        rise time     : 10% -> 90% 도달 시간
        peak time     : 최대/최소 peak가 발생한 시간
        overshoot     : target을 넘어간 양
        settling time : target 기준 ±1% step amplitude band에 들어가서 유지되기 시작한 시간

    Return:
        result = {
            "time": np.array(time_list),
            "position": np.array(position_list),
            "target_pos_mm": target_pos_mm,
            "command_time": command_time,

            "start_pos_mm": start_pos_mm,
            "step_amplitude_mm": step_amplitude_mm,

            "t_10_abs_sec": t_10_abs,
            "t_90_abs_sec": t_90_abs,
            "rise_time_sec": rise_time_sec,

            "peak_time_abs_sec": peak_time_abs,
            "peak_time_sec": peak_time_sec,
            "peak_value_mm": peak_value_mm,

            "overshoot_mm": overshoot_mm,
            "overshoot_percent": overshoot_percent,

            "settling_band_mm": settling_band_mm,
            "settling_time_abs_sec": settling_time_abs,
            "settling_time_sec": settling_time_sec,

            "actual_final_pos_mm": actual_final_pos_mm,
            "actual_error_mm": actual_error_mm,
            "success": success,
        }
    """

    def find_first_crossing_time(t_arr, y_arr, threshold, direction):
        """
        direction > 0: y가 threshold 이상이 되는 첫 시간
        direction < 0: y가 threshold 이하가 되는 첫 시간
        선형 보간으로 crossing time 계산
        """
        if len(t_arr) < 2:
            return None

        for i in range(1, len(t_arr)):
            t0 = t_arr[i - 1]
            t1 = t_arr[i]
            y0 = y_arr[i - 1]
            y1 = y_arr[i]

            if direction > 0:
                crossed = (y0 < threshold <= y1)
            else:
                crossed = (y0 > threshold >= y1)

            if crossed:
                if abs(y1 - y0) < 1e-12:
                    return t1

                ratio = (threshold - y0) / (y1 - y0)
                return t0 + ratio * (t1 - t0)

        return None

    def compute_settling_time(t_arr, y_arr, target, band, command_time):
        """
        command 이후 데이터에서,
        현재 시점 이후 모든 데이터가 target ± band 안에 들어오는 첫 시점을 찾음.
        """
        if len(t_arr) == 0:
            return None, None

        mask = t_arr >= command_time
        t_after = t_arr[mask]
        y_after = y_arr[mask]

        if len(t_after) == 0:
            return None, None

        inside = np.abs(y_after - target) <= band

        for i in range(len(t_after)):
            if np.all(inside[i:]):
                settling_time_abs = t_after[i]
                settling_time = settling_time_abs - command_time
                return settling_time_abs, settling_time

        return None, None

    def format_metric(value, unit="sec"):
        if value is None:
            return "None"
        return f"{value:.4f} {unit}"

    # -------------------------------------------------
    # Initial setting
    # -------------------------------------------------
    target_pos_mm = float(target_pos_mm)

    time_list = []
    position_list = []

    start_time = time.perf_counter()
    next_sample_time = start_time

    command_time = None
    actual_final_pos_mm = None
    actual_error_mm = None
    success = False

    print("[Gripper Position Step Response Recording] Starts")
    print(f"  target position = {target_pos_mm:.2f} mm")
    print(f"  current limit   = {current_ampere_mag:.4f} A")
    print(f"  pre record time = {pre_record_sec:.2f} sec")

    # -------------------------------------------------
    # 1. Record before command
    # -------------------------------------------------
    while True:
        now = time.perf_counter()
        elapsed = now - start_time

        if elapsed >= pre_record_sec:
            break

        actual_pos_mm = gripper.read_position(
            dxl_id=dxl_id,
            unit=RI.MM,
            homing_use=True,
            verbose=False,
        )

        if actual_pos_mm is not None:
            time_list.append(elapsed)
            position_list.append(actual_pos_mm)

        next_sample_time += sample_dt
        sleep_time = next_sample_time - time.perf_counter()

        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_sample_time = time.perf_counter()

    # -------------------------------------------------
    # Start position for step response
    # -------------------------------------------------
    if len(position_list) > 0:
        start_pos_mm = float(position_list[-1])
    else:
        start_pos_mm = gripper.read_position(
            dxl_id=dxl_id,
            unit=RI.MM,
            homing_use=True,
            verbose=False,
        )

        if start_pos_mm is None:
            logging.error("Failed to read start gripper position.")
            return {
                "time": np.array(time_list),
                "position": np.array(position_list),
                "target_pos_mm": target_pos_mm,
                "success": False,
            }

        start_pos_mm = float(start_pos_mm)

    step_amplitude_mm = target_pos_mm - start_pos_mm
    step_amplitude_abs_mm = abs(step_amplitude_mm)

    if step_amplitude_abs_mm < 1e-9:
        logging.warning("Step amplitude is almost zero.")
        settling_band_mm = 0.0
        direction = 1.0
    else:
        settling_band_mm = 0.01 * step_amplitude_abs_mm
        direction = 1.0 if step_amplitude_mm > 0.0 else -1.0

    level_10_mm = start_pos_mm + 0.1 * step_amplitude_mm
    level_90_mm = start_pos_mm + 0.9 * step_amplitude_mm

    print(f"  start position = {start_pos_mm:.2f} mm")
    print(f"  step amplitude = {step_amplitude_mm:.2f} mm")
    print(f"  1% band        = ±{settling_band_mm:.4f} mm")

    # -------------------------------------------------
    # 2. Send position command
    # -------------------------------------------------
    rv = gripper.current_based_position_control_linear(
        dxl_id=dxl_id,
        current_ampere_mag=current_ampere_mag,
        linear_pos=target_pos_mm,
        command_time_return=True,   # Necessary
    )

    if rv is False:
        logging.error("Failed to send gripper position command.")

        result = {
            "time": np.array(time_list),
            "position": np.array(position_list),
            "target_pos_mm": target_pos_mm,
            "command_time": command_time,
            "start_pos_mm": start_pos_mm,
            "step_amplitude_mm": step_amplitude_mm,
            "success": False,
        }

        return result
    
    # gripper.command_time은 time.perf_counter() 기준 absolute time
    if getattr(gripper, "command_time", None) is not None:
        command_time_abs = gripper.command_time
    else:
        logging.warning("gripper.command_time is None. Use current time instead.")
        command_time_abs = time.perf_counter()

    # plot/metric에 쓸 command_time은 start_time 기준 상대시간
    command_time = command_time_abs - start_time

    # command 시점의 start position을 한 점 추가
    time_list.append(command_time)
    position_list.append(start_pos_mm)

    logging.info(f"Position command sent at t = {command_time:.3f} sec")

    # -------------------------------------------------
    # 3. Record until settled
    # ------------------------------------------------- 
    command_start_time = command_time_abs
    settling_candidate_start_time = None
    next_sample_time = time.perf_counter()

    while True:
        now = time.perf_counter()
        elapsed = now - start_time
        elapsed_after_command = now - command_start_time

        if elapsed_after_command > timeout_sec:
            logging.warning("Position recording timeout.")
            break

        actual_pos_mm = gripper.read_position(
            dxl_id=dxl_id,
            unit=RI.MM,
            homing_use=True,
            verbose=False,
        )

        if actual_pos_mm is not None:
            time_list.append(elapsed)
            position_list.append(actual_pos_mm)

            actual_final_pos_mm = actual_pos_mm
            actual_error_mm = target_pos_mm - actual_pos_mm

            # 1% settling band 안에 들어왔는지 확인
            if abs(actual_error_mm) <= settling_band_mm:
                if settling_candidate_start_time is None:
                    settling_candidate_start_time = now

                # band 안에서 settled_time만큼 유지되면 완료
                if now - settling_candidate_start_time >= settled_time:
                    success = True
                    break

            else:
                settling_candidate_start_time = None

        next_sample_time += sample_dt
        sleep_time = next_sample_time - time.perf_counter()

        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_sample_time = time.perf_counter()

    # -------------------------------------------------
    # 4. Compute response metrics
    # -------------------------------------------------
    t_arr = np.array(time_list, dtype=np.float64)
    pos_arr = np.array(position_list, dtype=np.float64)

    command_mask = t_arr >= command_time
    t_after = t_arr[command_mask]
    pos_after = pos_arr[command_mask]

    t_10_abs = None
    t_90_abs = None
    rise_time_sec = None

    peak_time_abs = None
    peak_time_sec = None
    peak_value_mm = None

    overshoot_mm = None
    overshoot_percent = None

    settling_time_abs = None
    settling_time_sec = None

    if len(t_after) >= 2 and step_amplitude_abs_mm > 1e-9:
        # Rise time: 10% -> 90%
        t_10_abs = find_first_crossing_time(
            t_after,
            pos_after,
            level_10_mm,
            direction,
        )

        t_90_abs = find_first_crossing_time(
            t_after,
            pos_after,
            level_90_mm,
            direction,
        )

        if t_10_abs is not None and t_90_abs is not None:
            rise_time_sec = t_90_abs - t_10_abs

        # Peak time and overshoot
        # 증가 step이면 max, 감소 step이면 min을 peak/extreme으로 사용
        if direction > 0:
            peak_idx = int(np.argmax(pos_after))
            peak_value_mm = float(pos_after[peak_idx])
            overshoot_mm = max(0.0, peak_value_mm - target_pos_mm)
        else:
            peak_idx = int(np.argmin(pos_after))
            peak_value_mm = float(pos_after[peak_idx])
            overshoot_mm = max(0.0, target_pos_mm - peak_value_mm)

        peak_time_abs = float(t_after[peak_idx])
        peak_time_sec = peak_time_abs - command_time
        overshoot_percent = overshoot_mm / step_amplitude_abs_mm * 100.0

        # Settling time
        settling_time_abs, settling_time_sec = compute_settling_time(
            t_arr,
            pos_arr,
            target_pos_mm,
            settling_band_mm,
            command_time,
        )

    # -------------------------------------------------
    # 5. Result print
    # -------------------------------------------------
    if actual_final_pos_mm is not None:
        actual_error_mm = target_pos_mm - actual_final_pos_mm

        print(
            f"[Gripper Position Step Response Done]\n"
            f"  target              = {target_pos_mm:.2f} mm\n"
            f"  actual final        = {actual_final_pos_mm:.2f} mm\n"
            f"  final error         = {actual_error_mm:.2f} mm\n"
            f"  success             = {success}\n"
            f"\n"
            f"  t_10                = {format_metric(None if t_10_abs is None else t_10_abs - command_time)}\n"
            f"  t_90                = {format_metric(None if t_90_abs is None else t_90_abs - command_time)}\n"
            f"  rise time           = {format_metric(rise_time_sec)}\n"
            f"  peak time           = {format_metric(peak_time_sec)}\n"
            f"  peak value          = {format_metric(peak_value_mm, 'mm')}\n"
            f"  overshoot           = {format_metric(overshoot_mm, 'mm')}\n"
            f"  overshoot percent   = {format_metric(overshoot_percent, '%')}\n"
            f"  settling band       = ±{settling_band_mm:.4f} mm\n"
            f"  settling time       = {format_metric(settling_time_sec)}"
        )
    else:
        logging.error("No valid position data after command.")

    result = {
        "time": t_arr,
        "position": pos_arr,
        "target_pos_mm": target_pos_mm,
        "command_time": command_time,

        "start_pos_mm": start_pos_mm,
        "step_amplitude_mm": step_amplitude_mm,

        "t_10_abs_sec": t_10_abs,
        "t_90_abs_sec": t_90_abs,
        "rise_time_sec": rise_time_sec,

        "peak_time_abs_sec": peak_time_abs,
        "peak_time_sec": peak_time_sec,
        "peak_value_mm": peak_value_mm,

        "overshoot_mm": overshoot_mm,
        "overshoot_percent": overshoot_percent,

        "settling_band_mm": settling_band_mm,
        "settling_time_abs_sec": settling_time_abs,
        "settling_time_sec": settling_time_sec,

        "actual_final_pos_mm": actual_final_pos_mm,
        "actual_error_mm": actual_error_mm,
        "success": success,
    }

    # -------------------------------------------------
    # 6. Plot
    # -------------------------------------------------
    if plot and len(position_list) > 0:
        fig, ax = plt.subplots(figsize=(10, 6))
        
        ax.plot(
            t_arr, 
            pos_arr, 
            label="actual position [mm]", 
            color="blue", 
            linewidth=2.0
        )

        # target position
        ax.axhline(
            target_pos_mm,
            color="red",
            linestyle="--",
            linewidth=1.5,
            label=f"target position = {target_pos_mm:.2f} mm",
        )

        # command time
        if command_time is not None:
            ax.axvline(
                command_time,
                color="green",
                linestyle="--",
                linewidth=1.5,
                label=f"command sent = {command_time:.3f} sec",
            )

        # settling band
        if settling_band_mm > 0.0:
            ax.axhline(
                target_pos_mm + settling_band_mm,
                color="black",
                linestyle=":",
                linewidth=1.2,
                label="+1% settling band",
            )

            ax.axhline(
                target_pos_mm - settling_band_mm,
                color="black",
                linestyle=":",
                linewidth=1.2,
                label="-1% settling band",
            )
            
        # -------------------------------------------------
        # Metric label only
        # -------------------------------------------------
        metric_text = (
            "Step response metrics\n"
            f"Rise time          : {format_metric(rise_time_sec)}\n"
            f"Peak time          : {format_metric(peak_time_sec)}\n"
            f"Peak value         : {format_metric(peak_value_mm, 'mm')}\n"
            f"Overshoot          : {format_metric(overshoot_mm, 'mm')}\n"
            f"Overshoot percent  : {format_metric(overshoot_percent, '%')}\n"
            f"Settling band      : ±{settling_band_mm:.4f} mm\n"
            f"Settling time      : {format_metric(settling_time_sec)}"
        )

        ax.text(
            0.98,
            0.02,
            metric_text,
            transform=ax.transAxes,
            fontsize=11,
            verticalalignment="bottom",
            horizontalalignment="right",
            bbox=dict(
                boxstyle="round",
                facecolor="white",
                alpha=0.85,
            ),
        )

        ax.set_xlabel("Time [sec]")
        ax.set_ylabel("Position [mm]")
        ax.set_title(title)
        ax.grid(True)
        ax.legend(loc="center right")
        plt.tight_layout()
        plt.show()

    return result



