# Default Library for RBY1-A
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
import numpy as np
import time
from typing import Union
import signal
import sys



# =============================================================================
# Code
# =============================================================================
class RobotConfig:
    def __init__(self, mode: str, address: str , model: str):
        self.mode = mode
        self.address = address
        self.model = model



def setRobotConfig(operation_mode, operation_model) -> RobotConfig:
    """
    [Ex] robotConfig = ysw.setRobotConfig(ysw.MODE_SIM, ysw.MODEL_A)
    """ 
    # 1.Address Setting
    if operation_mode == RI.MODE_SIM:
        address = RI.ADDRESS_SIM
    elif operation_mode == RI.MODE_LOCAL_COMPUTER:
        address = RI.ADDRESS_UPC
    elif operation_mode == RI.MODE_LOCAL_COMPUTER_PML_WIFI:
        address = RI.ADDRESS_PML_WIFI
    elif operation_mode == RI.MODE_DIRECT_UPC:
        address = RI.ADDRESS_UPC
    else:
        logging.error("Invalid Address")
        return None
        
    # 2.Model Setting
    if operation_model not in [RI.MODEL_A, RI.MODEL_M, RI.MODEL_UB]:
        logging.error("Invalid Model")
        return None
            
    robotConfig = RobotConfig(operation_mode, address, operation_model)
    return robotConfig

            
            
def initialize(mode: str, address: str, model: str, power: str=".*", servo: str=".*", option: int=0) -> Union[rby.Robot_A, rby.Robot_M, rby.Robot_UB, None]:
    """
    [Ex] robot = ysw.initialize(robotConfig.mode, robotConfig.address, robotConfig.model, option=RI.CONNECT_AND_POWER)
    """ 
    UTIL.print_start_title("Initialization Starts")
    
    # 1.Create Robot Object
    robot = rby.create_robot(address, model)
    
    if (option in (RI.CONNECT_ONLY, RI.CONNECT_AND_POWER)):
        # 2.Connect to Robot
        if not robot.connect():                                     # Connect
            logging.error("Failed to connect to the Robot")
            return None
        logging.info("SUCCESS: Connected to the Robot!")
    
    if (option == RI.CONNECT_AND_POWER):
        # 3.Power ON
        if not robot.is_power_on(power):                            # Power Check
            if not robot.power_on(power):                           # Power ON
                logging.error("Failed to Power ON")
                return None
        logging.info("SUCCESS: Power ON the Robot!")

        # 4.Servo ON
        if not robot.is_servo_on(servo):                            # Servo Check
            if not robot.servo_on(servo):                           # Servo ON
                logging.error("Failed to Servo ON")
                return None
        logging.info("SUCCESS: Servo ON the Robot!")
    
        # 5.Reset Control Manager
        if robot.get_control_manager_state().state in [             # CM Check
            rby.ControlManagerState.State.MajorFault, 
            rby.ControlManagerState.State.MinorFault
        ]:
            if not robot.reset_fault_control_manager():             # CM Reset
                logging.error("Failed to Reset the Control Manager")
                return None
            # After the fault, enable/disable CM is not working without "unlimited_mode_enabled=True"
            if not robot.enable_control_manager(unlimited_mode_enabled=True):       # Essential for Reinitialize from Fault
                logging.error("Failed to Unlimit the Control Manager after Fault")
                return None

        logging.info("SUCCESS: Reset the Control Manager!")
    
        # 6.Enable Control Manager
        if not robot.enable_control_manager():                      # CM Enable
            logging.error("Failed to Enable the Control Manager")
            return None
        logging.info("SUCCESS: Enable the Control Manager!")
        
        # 7.Enable Grippers (Only for UPC mode)
        if mode == RI.MODE_DIRECT_UPC:
            if not robot.set_tool_flange_output_voltage("right", RI.TOOL_FLANGE_VOLTAGE_ON):   # Power ON R-gripper
                logging.error("Failed to Enable the Right Gripper")
                return None
            if not robot.set_tool_flange_output_voltage("left", RI.TOOL_FLANGE_VOLTAGE_ON):    # Power ON L-gripper
                logging.error("Failed to Enable the Left Gripper")
                return None
            logging.info("SUCCESS: Enable the Grippers!")
        
    UTIL.print_end_title("Intialization is Done")
    return robot
        
        
        
def powerOFF(robot, robotConfig: RobotConfig) -> bool:
    """
    [Ex] ysw.powerOFF(robot, robotConfig)
    """ 
    
    # 0.Check Power State
    if not robot.is_power_on(".*"): 
        logging.info("Already Power OFF")
        return True
    
    UTIL.print_start_title("powerOFF Starts")
    
    # 1.Disable Grippers (Only for UPC mode)
    if robotConfig.mode == RI.MODE_DIRECT_UPC:
        if not robot.set_tool_flange_output_voltage("right", RI.TOOL_FLANGE_VOLTAGE_OFF):
            logging.error("Failed to Disable Right Gripper")
            return False 
        if not robot.set_tool_flange_output_voltage("left", RI.TOOL_FLANGE_VOLTAGE_OFF):
            logging.error("Failed to Disable Left Gripper")
            return False 
        logging.info("SUCCESS: Disable Grippers!")
    
    # 2.Disable Control Managuer
    if not robot.disable_control_manager():               
        logging.error("Failed to Disable the Control Manager")
        return False
    logging.info("SUCCESS: Disable the Control Manager!")
        
    # 3.Servo OFF
    # if robot.is_servo_on(".*"):                            # Servo Check
    #     if not robot.servo_off(".*"):                      # 함수는 있는데 실행 안됨
    #         logging.error("Failed to Servo OFF")
    #         return False
    # logging.info("SUCCESS: Servo OFF the robot!")
        
    # 4.Power OFF
    if robot.is_power_on(".*"):                           # Power Check
        if not robot.power_off(".*"):                     # Power ON
            logging.error("Failed to Power OFF")
            return False
            
    UTIL.print_end_title("PowerOFF is Done")
    return True

        

def getInfo(robot):
    """
    [Ex] ysw.getInfo(robot)
    """ 
    UTIL.print_start_title("Robot Info Starts")
    
    robot_info = robot.get_robot_info()
    print(repr(robot_info))
    
    UTIL.print_end_title("Robot Info Ends")
    
    
    
def getState(robot):
    """
    [Ex] ysw.getState(robot)
    """ 
    UTIL.print_start_title("Robot State Starts")
    
    robot_state = robot.get_state()
    print(repr(robot_state))
    
    UTIL.print_end_title("Robot State Ends")    
    
    
    
def getStateStream(robot):
    """
    [Ex] ysw.getStateStream(robot)
    """ 
    def callback(robot_state, control_manager_state: rby.ControlManagerState):
        np.set_printoptions(precision=3, suppress=True, floatmode="fixed")
        
        UTIL.print_start_title("Robot State Starts")
        print(robot_state)
        UTIL.print_end_title("Robot State Ends")  
        
        UTIL.print_start_title("Control Manager State Starts")  
        print(control_manager_state)
        UTIL.print_end_title("Control Manager State Ends")  
        
    robot.start_state_update(callback, rate=10)     # Hz
    try:
        time.sleep(100)         # sec
    except KeyboardInterrupt:
        UTIL.print_start_title("Stopping State Streaming...")
    finally:
        robot.stop_state_update()
        UTIL.print_end_title("State Streaming Ends")
    


def getURDF(robot):
    """
    [Ex] ysw.getURDF(robot)
    """ 
    UTIL.print_start_title("URDF Starts")
    
    urdf_info = robot.get_robot_model()
    print(urdf_info)
        
    UTIL.print_end_title("URDF Ends")
    


def getParameterList(robot):
    """
    [Ex] ysw.getParameterList(robot)
    """ 
    UTIL.print_start_title("Parameter List Starts")
    
    parameter_list = robot.get_parameter_list()
    for parameter in parameter_list:
        print(parameter)
        
    UTIL.print_end_title("Parameter List Ends")
    
    

def getParameterValue(robot, parameter_name):
    """
    [Ex] ysw.getParameterValue(robot, "joint_position_command.cutoff_frequency")
    """  
    parameter_value = robot.get_parameter(parameter_name)
    return parameter_value



def getToolFlangeStateStream(robot):
    """
    [Ex] ysw.getToolFlangeStateStream(robot)
    """ 
    def callback(robot_state):
        UTIL.print_start_title("Right Tool Flange Starts")
        print(robot_state.tool_flange_right)
        UTIL.print_end_title("Right Tool Flange Ends")  
        
        UTIL.print_start_title("Left Tool Flange Starts")  
        print(robot_state.tool_flange_left)
        UTIL.print_end_title("Left Tool Flange Ends")  
        
        
    robot.start_state_update(callback, rate=10)     # Hz
    try:
        time.sleep(100)         # sec
    except KeyboardInterrupt:
        UTIL.print_start_title("Stopping State Streaming...")
    finally:
        robot.stop_state_update()
        UTIL.print_end_title("State Streaming Ends")
        
    

def checkFWversion(robot):
    """
    [Ex] ysw.checkFWversion(robot)
    """ 
    UTIL.print_start_title("FW Version Starts")
    
    print(f"{'[Joint Name]':<20} {'[Has Brake]':<15} {'[Product Name]':<20} {'[FW Version]':<15}")
    
    robot_info = robot.get_robot_info()
    for ji in robot_info.joint_infos:
        print(
            f"{ji.name:<20} "
            f"{str(ji.has_brake):<15} "
            f"{ji.product_name:<20} "
            f"{ji.firmware_version:<15}"
        )
        
    UTIL.print_end_title("FW Version Ends")
    
    
    
def setLED(robot, rgb, duration, transition_time, blinking, blinking_freq):
    """
    [Ex] ysw.setLED(robot, ysw.LED_RED, 0.5, 0.1, False)
    """ 
    if (blinking == False):
        robot.set_led_color(rby.Color(*rgb), duration, transition_time, blinking)
        time.sleep(duration)
    else:
        robot.set_led_color(rby.Color(*rgb), duration, transition_time, blinking, blinking_freq)
        time.sleep(duration)
        
        
        
        

class Recorder:
    """
    Robot should be 'Initialized' before you use this Class 
    """
    def __init__(self, robot, filename: str="record_data.npz", rate: int=10):
        self.robot = robot
        self.filename = filename
        self.rate = rate
        self.record_list = []
        self.record_flag = False
        self.is_state_update_started = False
        self.saved = False
        
        
    def callback(self, state):
        if self.record_flag:
            print("---")
            print(f"Target position: {state.target_position}")
            print(f"Current position: {state.position}")
            current_state = state.position.copy()
            self.record_list.append(current_state)
            
            
    def save(self):
        if not self.saved:
            np.savez_compressed(self.filename, data=np.asarray(self.record_list))
            self.saved = True
            print(f"Recording stopped and data saved to '{self.filename}'.")
            print(f"Recorded samples: {len(self.record_list)}")
    
    
    def start_record(self):
        self.record_list = []
        self.record_flag = True
        self.saved = False
        
        if not self.is_state_update_started:
            self.robot.start_state_update(self.callback, self.rate)
            self.is_state_update_started = True
        
        UTIL.print_start_title("Recording Started...")
        
        
    def stop_record(self):
        self.record_flag = False
        
        if self.is_state_update_started:
            self.robot.stop_state_update()
            self.is_state_update_started = False
        
        UTIL.print_end_title("Recording Stopped")
        self.save()
        
        
    def handle_exit(self, sig, frame):
        """ Handler called when the program exits. """
        self.stop_record()
        sys.exit(0)
        
        
    def run_record(self):
        signal.signal(signal.SIGINT, self.handle_exit)      # SIGINT  : Ctrl+C
        signal.signal(signal.SIGTERM, self.handle_exit)     # SIGTERM : termination signal
        
        self.start_record()
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            self.handle_exit()
        
        