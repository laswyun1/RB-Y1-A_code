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
                
            # 4. Enable Torque mode
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
    
    
    
    def current_based_position_control_linear(self, dxl_id: int, current_ampere_mag: float=0.02, linear_pos: float=0.0) -> bool:
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
        
        # 4. Set Goal Position
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
    move_time,
    current_ampere_mag=0.02,
    v_max=40.0,      # [mm/s]
    a_max=50.0,      # [mm/s^2]
    dt=0.01,         # [sec], 100Hz
    pos_tol=1.0,     # [mm]
    max_time=10.0,   # [sec]
    plot=True,
):
    """
        Time-based trapezoidal trajectory generation and position control.
        
        x0 -> target_pos_mm까지 move_time초 동안 이동하도록
        trapezoidal velocity profile을 먼저 설계하고,
        그에 따라 position command x_cmd(t)를 생성해서 gripper에 보냄.

        x     : commanded position [mm]
        v     : commanded velocity [mm/s]
        a     : commanded acceleration [mm/s^2]

        start_pos_mm  -> target_pos_mm 로 이동하면서
            |v| <= v_max
            |a| <= a_max
        를 만족하도록 중간 position command를 계속 보냄.
        
        Return:
            True  : target reached
            False : failed or timeout
    """
    
    def plot_trapezoidal_profile():
        if not plot:
            return

        if len(t_cmd_list) == 0:
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
        
        
    # Read start position
    start_pos_mm = gripper.read_position(
        dxl_id=dxl_id,
        unit=RI.MM,
        homing_use=True,
        print_log=False,
    )
    
    if start_pos_mm is None:
        logging.error(f"Failed to read gripper position. dxl_id={dxl_id}")
        return False
    
    x = float(start_pos_mm)
    target_pos_mm = float(target_pos_mm)
    
    v = 0.0
    a = 0.0
    start_time = time.time()
    
    v_max = abs(float(v_max))
    a_max = abs(float(a_max))
    dt = float(dt)
    
    # For Safety
    if v_max <= 0.0:    raise ValueError("v_max must be positive.")
    if a_max <= 0.0:    raise ValueError("a_max must be positive.")
    if dt <= 0.0:       raise ValueError("dt must be positive.")
    
    # Data for plotting
    t_cmd_list = [0.0]
    x_cmd_list = [x]
    v_cmd_list = [v]
    a_cmd_list = [a]
    
    print(f"[Gripper Trapezoidal profile] start={x:.2f} mm -> target={target_pos_mm:.2f} mm")
    
    while True:
        elapsed = time.time() - start_time
        
        # Timeout check 
        if elapsed > max_time:
            logging.error("trapezoidal_position_command timeout.")
            plot_trapezoidal_profile()
            return False
        
        # Remaining distance
        error = target_pos_mm - x
        
        # Terminate condition
        if abs(error) <= pos_tol and abs(v) <= vel_tol:
            x = target_pos_mm
            v = 0.0
            a = 0.0
            
            elapsed = time.time() - start_time
            
            t_cmd_list.append(elapsed)
            x_cmd_list.append(x)
            v_cmd_list.append(v)
            a_cmd_list.append(a)
            
            rv = gripper.current_based_position_control_linear(
                dxl_id,
                current_ampere_mag=current_ampere_mag,
                linear_pos=x
            )
            
            if rv is False:
                logging.error("Failed to send final gripper command")
                plot_trapezoidal_profile()
                return False
        
            time.sleep(0.2)
            
            # 실제 위치 확인
            actual_pos_mm = gripper.read_position(
                dxl_id=dxl_id,
                unit=RI.MM,
                homing_use=True,
                print_log=False,
            )
            
            if actual_pos_mm is None:
                logging.error("Failed to read final gripper position")
                plot_trapezoidal_profile()
                return False
            
            actual_error = target_pos_mm - actual_pos_mm
            
            print(
                f"[Gripper Trapezoid Done] "
                f"target={target_pos_mm:.2f} mm, "
                f"actual={actual_pos_mm:.2f} mm, "
                f"error={actual_error:.2f} mm"
            )
            
            plot_trapezoidal_profile()
            
            if abs(actual_error) <= pos_tol:
                return True
            else:
                logging.info("Command finished, but actual position is not close enough.")
                return False

        # -------------------------------------------------
        # Trajectory Generator
        # -------------------------------------------------
        direction_to_goal = sign(error)
        velocity_direction = sign(v)                    # 현재 움직이는 방향
        stopping_distance = (v**2) / (2.0 * a_max)      # 현재 속도로부터 필요한 stopping distance
        
        # 1. 목표 반대 방향으로 움직이고 있다면 먼저 감속
        if velocity_direction != 0.0 and velocity_direction != direction_to_goal:
            a = -velocity_direction * a_max

        # 2. 이제 braking을 시작해야 할 경우
        elif abs(error) <= stopping_distance:
            if velocity_direction != 0.0:
                a = -velocity_direction * a_max
            else:
                a = 0.0

        # 3. 아직 충분한 거리가 남음 -> 가속
        else:
            if abs(v) < v_max:
                a = direction_to_goal * a_max
            else:
                a = 0.0

        # velocity update
        v_new = v + a * dt
        v_new = max(-v_max, min(v_max, v_new))      # velocity saturation

        # position update
        x_new = x + 0.5 * (v + v_new) * dt

        # 목표를 지나쳐버리는 numerical overshoot 방지
        if (target_pos_mm - x) * (target_pos_mm - x_new) < 0.0:
            x_new = target_pos_mm
            v_new = 0.0
            a = 0.0

        x = x_new
        v = v_new
        
        elapsed = time.time() - start_time

        t_cmd_list.append(elapsed)
        x_cmd_list.append(x)
        v_cmd_list.append(v)
        a_cmd_list.append(a)

        # -------------------------------------------------
        # Dynamixel command
        # -------------------------------------------------
        rv = gripper.current_based_position_control_linear(
            dxl_id,
            current_ampere_mag=current_ampere_mag,
            linear_pos=x
        )
        if rv is False:
            logging.error("Failed to send gripper position command")
        
        time.sleep(dt)


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