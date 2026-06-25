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
        self.port_hdlr = None 
        self.packet_hdlr = None
        self.dxl_R_idx = 0
        self.dxl_L_idx = 1
        self.dxl_activated = []
        
        
    def connect(self, device_name="/dev/rby1_gripper", protocol_version=2.0, baudrate=2000000):
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


    def test_ping(self):
        UTIL.print_start_title("Gripper Ping Test Starts")
        self.dxl_activated = []
        
        for dxl_id in [RI.GRIPPER_ID_R, RI.GRIPPER_ID_L]:
            dxl_model_num, dxl_comm_res, dxl_err = self.packet_hdlr.ping(self.port_hdlr, dxl_id)
            
            if dxl_comm_res != RI.DXL_COMM_SUCCESS:
                logging.error("[DXL ID:%d] %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
            elif dxl_err != RI.DXL_NO_ERROR:
                logging.error("[DXL ID:%d] %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
            else:
                self.dxl_activated.append(dxl_id)
                logging.info("[DXL ID:%d] SUCCESS: Dynamixel model number is %d" % (dxl_id, dxl_model_num))
                
        UTIL.print_end_title("Gripper Ping Test is Done")
                
                
    def write_torque_mode(self, dxl_id: int, torque_mode: int):
        dxl_comm_res, dxl_err = self.packet_hdlr.write1ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_TORQUE_ENABLE,
            int(torque_mode)
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
        else:
            if (torque_mode == RI.DXL_TORQUE_DISABLE):      logging.info("SUCCESS: Torque is Disabled")
            elif (torque_mode == RI.DXL_TORQUE_ENABLE):     logging.info("SUCCESS: Torque is Enabled")
                
   
    def read_torque_mode(self, dxl_id: int) -> Optional[int]:
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
        
        
    def write_operation_mode(self, dxl_id: int, operation_mode: int):
        dxl_comm_res, dxl_err = self.packet_hdlr.write1ByteTxRx(
            self.port_hdlr,
            dxl_id,
            RI.DXL_ADDR_OPERATING_MODE,
            int(operation_mode)
        )
        if dxl_comm_res != RI.DXL_COMM_SUCCESS:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getTxRxResult(dxl_comm_res)))
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
        else:
            if (operation_mode == RI.DXL_CURRENT_CONTROL_MODE):      logging.info("SUCCESS: Set Current Control Mode")
            elif (operation_mode == RI.DXL_POSITION_CONTROL_MODE):   logging.info("SUCCESS: Set Postion Control Mode")
            
            
    def read_operation_mode(self, dxl_id: int) -> Optional[int]:
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
            return operation_mode    # 0: Current Control, 3: Position Control  


    def read_position_rad(self, dxl_id: int) -> Optional[float]:
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
            position_rad = float(val) / RI.DXL_RESOLUTION_TICKS_PER_REV * RI.TWO_PI 
            logging.info(f"[DXL ID:{dxl_id}] SUCCESS: position = {position_rad}rad")
            return position_rad    # [rad]
        
        
    def write_current_ampere(self, dxl_id: int, current_ampere: float):
        """1-unit = 2.69mA"""        
        current_value = int(current_ampere / RI.DXL_AMPERE_PER_UNIT)
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
            logging.info(f"[DXL ID:{dxl_id}] SUCCESS: Set Current [{current_ampere} A]")


    def write_current_ampere_safety(self, dxl_id: int, current_ampere: float):        
        """1-unit = 2.69mA / Ensure the connection"""
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
                logging.info(f"[DXL ID:{dxl_id}] SUCCESS: Set Current [{current_ampere} A]")
                time.sleep(0.5)
                break
            print(f"[DXL ID]:{dxl_id} Retry...")
            
            
    def read_current_ampere(self, dxl_id: int) -> Optional[float]:        
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
            logging.info(f"[DXL ID:{dxl_id}] SUCCESS: current = {current_A}A")
            return current_A     # [A]
        
        
    def read_current_limit(self, dxl_id: int) -> Optional[float]:  
        """Range (0~1193)"""      
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
            logging.info(f"[DXL ID:{dxl_id}] SUCCESS: current_limit_raw(0~1193) = {current_limit_raw}, current_limit_A = {current_limit_A}A")
            return current_limit_A     # [A]
        

    def write_current_limit(self, dxl_id: int, current_limit_A: float) -> None:        
        """Range (0~1193)"""
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
        elif dxl_err != RI.DXL_NO_ERROR:
            logging.error("[DXL ID:%d] Fail %s" % (dxl_id, self.packet_hdlr.getRxPacketError(dxl_err)))
        else:
            logging.info(f"SUCCESS: Set Current Limit: {current_limit_A}A")