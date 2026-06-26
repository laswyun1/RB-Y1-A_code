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
        
        
    def read_position_angular(self, dxl_id: int, unit=RI.RAD, print_log=True) -> Optional[float]:
        """
            Read current Angle Position  
            Default Gripper Range(Approximation): -170deg(open) ~ +90deg(closed) 
            [Ex] gripper.read_position_angular(dxl_id=ysw.GRIPPER_ID_R, unit=ysw.DEG, print_log=True)
            
            Return:
                angle position (unit = radian or degree)
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
                
            position_rad = float(signed_val) / RI.DXL_RESOLUTION_TICKS_PER_REV * RI.TWO_PI
            position_deg = position_rad * RI.RAD2DEG
            if (unit == RI.RAD):
                if print_log == True: 
                    logging.info(f"SUCCESS: [DXL ID:{dxl_id}] position = {position_rad} rad")
                return position_rad    # [rad]
            elif (unit == RI.DEG):
                if print_log == True:
                    logging.info(f"SUCCESS: [DXL ID:{dxl_id}] position = {position_deg} deg")
                return position_deg    # [deg]
            
            
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
            elif (operation_mode == RI.DXL_CURRENT_BASED_POSITION_CONTROL_MODE):   
                logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set (Current + Postion) Control Mode")
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
        current_value = int(current_ampere / RI.DXL_AMPERE_PER_UNIT)
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
            logging.info(f"SUCCESS: [DXL ID:{dxl_id}] Set Current [{current_ampere} A]")
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
            [Ex] gripper.write_current_limit(ysw.GRIPPER_ID_R, ysw.DXL_CURRENT_LIMIT_MAX_A)
            
            Return: 
                True: OK
                False: Error
        """  
        current_limit_raw = int(current_limit_A / RI.DXL_AMPERE_PER_UNIT)
        if  (current_limit_raw > 1193):     current_limit_raw = 1193

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
            self.write_current_limit(dxl_id=dxl_id, current_limit_A=RI.DXL_CURRENT_LIMIT_MAX_A)
            
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