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
        
        
        
    def read_position(self, dxl_id: int, unit=RI.RAD, homing_use=True, print_log=True) -> Optional[float]:
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
                if print_log == True: 
                    logging.info(f"SUCCESS: [DXL ID:{dxl_id}] position = {position_rad} rad")
                return position_rad    # [rad]
            elif (unit == RI.DEG):
                if print_log == True:
                    logging.info(f"SUCCESS: [DXL ID:{dxl_id}] position = {position_deg} deg")
                return position_deg    # [deg]
            elif (unit == RI.TICK):
                if print_log == True:
                    logging.info(f"SUCCESS: [DXL ID:{dxl_id}] position = {position_tick} tick")
                return position_tick    # [tick]
            elif (unit == RI.MM):
                if (homing_use == True):
                    if print_log == True:
                        logging.info(f"SUCCESS: [DXL ID:{dxl_id}] position = {position_mm} mm")
                    return position_mm    # [mm]   
                else:
                    logging.error("For [mm] unit, homing must be used")
                    return None
            
            
            
    def read_current_ampere(self, dxl_id: int) -> Optional[float]:    
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
            current_A = float(val) * RI.DXL_AMPERE_PER_UNIT 
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
        
        
        
    def read_PIDgain_posCtrl(self, dxl_id: int) -> list[float]:  
        """
            Read PID gain of position control
            Raw value Range: (0 ~ 16,383)
            [Ex] gripper.read_PIDgain_posCtrl(dxl_id=ysw.GRIPPER_ID_R)
            
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
        
        if (current_ampere > 0.6725):
            logging.info(f"[DXL ID:%d] Current MUST be less than 672.5mA")
            logging.info(f"[DXL ID:%d] Thus, Maximum Currentt(672.5mA) is applied")
            current_ampere_revised = 0.6725
            
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
        current_value = int(current_ampere / RI.DXL_AMPERE_PER_UNIT)
        
        while True:
            dxl_comm_res, dxl_err = self.packet_hdlr.write2ByteTxRx(
                self.port_hdlr,
                dxl_id,
                RI.DXL_ADDR_GOAL_CURRENT,
                current_value
            )
            if dxl_comm_res != RI.DXL_COMM_SUCCESS:
                logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
                time.sleep(0.5)
            elif dxl_err != RI.DXL_NO_ERROR:
                logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
                time.sleep(0.5)
            else:
                logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set Current [{current_ampere} A]")
                time.sleep(0.5)
                break
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
        
        
    def write_PIDgain_posCtrl(self, dxl_id: int, Kp: int=RI.DXL_P_GAIN_POSCTRL, Ki: int=RI.DXL_I_GAIN_POSCTRL, Kd: int=RI.DXL_D_GAIN_POSCTRL) -> bool:        
        """
            Set P,I,D gain for DXL
            Raw value Range (0 ~ 16,383)
            [Ex] gripper.write_PIDgain_posCtrl(ysw.GRIPPER_ID_R, ysw.DXL_P_GAIN_POSCTRL, ysw.DXL_I_GAIN_POSCTRL, ysw.DXL_D_GAIN_POSCTRL)
            
            Return: 
                True: OK
                False: Error
        """  
        if (Kp < 0 or Ki < 0 or Kd < 0):     
            logging.error("Kp, Ki, Kd values cannot be less than 0")
            return False
        if (Kp > RI.DXL_PID_GAIN_LIMIT or Ki > RI.DXL_PID_GAIN_LIMIT or Kd > RI.DXL_PID_GAIN_LIMIT):     
            logging.error("Kp, Ki, Kd values cannot exceed limit(=1000)")
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
    def initialize_setting(self, dxl_id=None, operation_mode=RI.DXL_CURRENT_CONTROL_MODE) -> bool:
        """
            Initialize Gripper (Set Max Current + Set Operation Mode)
            This function should be used after "connect" function is used
            [Ex] gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
            
            Return:
                True: succesfully initialized
                False: not initialized
        """
        UTIL.print_start_title(f"Gripper [DXL ID:{dxl_id}] setting Starts")
        
        if ((dxl_id == None) or (dxl_id not in [RI.GRIPPER_ID_R, RI.GRIPPER_ID_L])):
            logging.error("Invalid DXL ID")
            return False
        if dxl_id in self.dxl_activated:
            self.write_torque_mode(dxl_id=dxl_id, torque_mode=RI.DXL_TORQUE_DISABLE)
            # self.write_current_limit(dxl_id=dxl_id, current_limit_A=RI.DXL_CURRENT_LIMIT_MAX_A_YSW)  # You Never use this (You don't need to set current limit)
            
            if operation_mode in RI.DXL_CONTROL_MODE:
                self.write_operation_mode(dxl_id=dxl_id, operation_mode=operation_mode)
            else:
                logging.error("Invalid DXL Operation Mode")
                return False
                
            self.write_torque_mode(dxl_id=dxl_id, torque_mode=RI.DXL_TORQUE_ENABLE)
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




    def tune_PIDgain_posCtrl(self, dxl_id: int, Kp: int=RI.DXL_P_GAIN_POSCTRL, Ki: int=RI.DXL_I_GAIN_POSCTRL, Kd: int=RI.DXL_D_GAIN_POSCTRL) -> bool:
        """
            Setting the PID gain of Position Control of Gripper
            After use this function, the setting is maintained.
                
            [Ex]
                gripper.tune_PIDgain_posCtrl(ysw.GRIPPER_ID_R, 800, 10, 0)   # close
        """
        UTIL.print_start_title(f"Gripper [DXL ID:{dxl_id}] Tuning PID gain for Position Control Starts")
        
        if (dxl_id is None) or (dxl_id not in [RI.GRIPPER_ID_R, RI.GRIPPER_ID_L]):
            logging.error("Invalid DXL ID")
            return False

        if dxl_id not in self.dxl_activated:
            logging.error(f"[DXL ID:{dxl_id}] is NOT activated")
            return False
        
        [Kp_old, Ki_old, Kd_old] = self.read_PIDgain_posCtrl(dxl_id)
        logging.info(f"[DXL ID:{dxl_id}] Original PID gain: Kp={Kp_old}, Ki={Ki_old}, Kd={Kd_old}")
        
        self.write_PIDgain_posCtrl(dxl_id, Kp, Ki, Kd)
        
        [Kp_new, Ki_new, Kd_new] = self.read_PIDgain_posCtrl(dxl_id)
        logging.info(f"[DXL ID:{dxl_id}] Newly tuned PID gain: Kp={Kp_new}, Ki={Ki_new}, Kd={Kd_new}")
         
        UTIL.print_end_title(f"Gripper [DXL ID:{dxl_id}] Tuning PID gain for Position Control Ends")
        return True
