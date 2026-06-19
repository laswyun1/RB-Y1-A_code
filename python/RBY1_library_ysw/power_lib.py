import rby1_sdk as rby
import robot_info as RI
import logging
from typing import Union


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

            
            
            
def initialize(mode, address, model, power=".*", servo=".*") -> Union[rby.Robot_A, rby.Robot_M, rby.Robot_UB]:
    """
    [Ex] robot = ysw.initialize(robotConfig.mode, robotConfig.address, robotConfig.model)
    """ 
    # 1.Create Robot Object
    robot = rby.create_robot(address, model)
    
    # 2.Connect to Robot
    if not robot.connect():                                     # Connect
        logging.error("Failed to connect to robot")
        return None
    logging.info("SUCCESS: Connected to the robot!")
    
    # 3.Power ON
    if not robot.is_power_on(power):                            # Power Check
        if not robot.power_on(power):                           # Power ON
            logging.error("Failed to Power ON")
            return None
    logging.info("SUCCESS: Power ON the robot!")

    # 4.Servo ON
    if not robot.is_servo_on(servo):                            # Servo Check
        if not robot.servo_on(servo):                           # Servo ON
            logging.error("Failed to Servo ON")
            return None
    logging.info("SUCCESS: Servo ON the robot!")
    
    # 5.Reset Control Manager
    if robot.get_control_manager_state().state in [             # CM Check
        rby.ControlManagerState.State.MajorFault, 
        rby.ControlManagerState.State.MinorFault
    ]:
        if not robot.reset_fault_control_manager():             # CM Reset
            logging.error("Failed to Reset control manager")
            return None
    logging.info("SUCCESS: Reset the control manager!")
    
    # 6.Enable Control Manager
    if not robot.enable_control_manager():                      # CM Enable
        logging.error("Failed to Enable control manager")
        return None
    logging.info("SUCCESS: Enable the control manager!")
        
    # 7.Enable Grippers (Only for UPC mode)
    if mode == RI.MODE_DIRECT_UPC:
        if not robot.set_tool_flange_output_voltage("right", RI.TOOL_FLANGE_VOLTAGE_ON):   # Power ON R-gripper
            logging.error("Failed to Enable Right gripper")
            return None
        if not robot.set_tool_flange_output_voltage("left", RI.TOOL_FLANGE_VOLTAGE_ON):    # Power ON L-gripper
            logging.error("Failed to Enable Left gripper")
            return None
        logging.info("SUCCESS: Enable Grippers!")
    
    logging.info("\n*********** [Intialization is Done] ***********")
    return robot
        
        
        
def powerOFF(robot, robotConfig: RobotConfig) -> bool:
    # 1.Disable Grippers (Only for UPC mode)
    if robotConfig.mode == RI.MODE_DIRECT_UPC:
        if not robot.set_tool_flange_output_voltage("right", RI.TOOL_FLANGE_VOLTAGE_OFF):
            logging.error("Failed to Disable Right gripper")
            return False 
        if not robot.set_tool_flange_output_voltage("left", RI.TOOL_FLANGE_VOLTAGE_OFF):
            logging.error("Failed to Disable Left gripper")
            return False 
        logging.info("SUCCESS: Disable Grippers!")
    
    # 2.Disable Control Managuer
    if not robot.disable_control_manager():               
        logging.error("Failed to Disable control manager")
        return False
        
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
            
    logging.info("\n*********** [PowerOFF is Done] ***********")
    return True
    
        







