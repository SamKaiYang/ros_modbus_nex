#!/usr/bin/env python
import rospy
from modbus.modbus_wrapper_client import ModbusWrapperClient 
from std_msgs.msg import Int32MultiArray as HoldingRegister
import time

WRITE_NUM_REGISTERS = 36
READ_INPUT_NUM_REGISTERS = 272
ADDRESS_WRITE_START = 4096
ADDRESS_READ_INPUT_START = 8192
class ModbusNexApi():
    def __init__(self):
        self.host = "192.168.0.6"
        self.port = 502
        self.modclient = ModbusWrapperClient(self.host,self.port)

        self.modclient.setWritingRegisters(ADDRESS_WRITE_START,WRITE_NUM_REGISTERS)
        # self.modclient.setReadingRegisters(ADDRESS_READ_START,READ_NUM_REGISTERS)
        self.modclient.setReadingInputRegisters(ADDRESS_READ_INPUT_START,READ_INPUT_NUM_REGISTERS)
        rospy.loginfo("Setup complete")

    def read_holding_Listening(self):
        self.modclient.startListening()
        rospy.loginfo("read_holding Listener started")

    def read_input_Listening(self):
        self.modclient.start_readinput_Listening()
        rospy.loginfo("read_input Listener started")


    # ----------------Write msg to modbus server API-------------------
    def start_programs(self):
        """
            Starting loaded programs (rising edge)
        """
        register = 4096
        value = 5
        timeout = 0.5
        self.modclient.setOutput(register,value,0)
        rospy.sleep(timeout)
        rospy.loginfo("START")

    # TODO: when start programs is set, this function is only available
    def stop_programs(self):
        """
            Stopped running programs (falling edge)
        """
        register = 4096
        value = 4
        timeout = 0.5
        self.modclient.setOutput(register,value,0)
        rospy.sleep(timeout)
        rospy.loginfo("STOP")

    def enable_robot(self):
        """
            Enable robot (rising edge)
        """
        register = 4096
        value = 4
        timeout = 0.5
        self.modclient.setOutput(register,value,0)
        rospy.sleep(timeout)
        rospy.loginfo("ENABLE")

    # TODO: when programs is stop or not use, this function is only available
    def disable_robot(self):
        """
            Disable robot (falling edge)
        """
        register = 4096
        value = 0
        timeout = 0.5
        self.modclient.setOutput(register,value,0)
        rospy.sleep(timeout)
        rospy.loginfo("DISABLE")
        
    def reload_all_programs(self):
        """
            Reload all programs (*2)
        """
        register = 4096
        value = 8
        timeout = 0.5
        self.modclient.setOutput(register,value,0)
        rospy.sleep(timeout)
        rospy.loginfo("RELOAD_ALL")

    def reload_sel_programs(self):
        """
            Reload selected programs according to register:1001h (*3)
        """
        register = 4096
        value = 16
        timeout = 0.5
        self.modclient.setOutput(register,value,0)
        rospy.sleep(timeout)
        rospy.loginfo("RELOAD_SEL")
        
    def shutdown_controller(self):
        """
            Shutdown controller
        """
        register = 4096
        value = 1024
        timeout = 0.5
        self.modclient.setOutput(register,value,0)
        rospy.sleep(timeout)
        rospy.loginfo("SHUTDOWN")

    # ----------------Request modbus server to read server state API-------------------
    def operation_mode_state(self):
        """
            (0)T1, (1)T2, (2)AUT, (3)EXT
        """
        input_registers = self.modclient.read_input_Registers(8192,1)
        return input_registers

    def safety_state(self):
        """
            (0)Disable, (1)Ready, (2)Error, (3)Enable, (4)Running
        """
        input_registers = self.modclient.read_input_Registers(8193,1)
        return input_registers

    def enable_switch_state(self):
        """
            (0)Disable,(1)Enable, (2)Pressed-EMG
        """
        input_registers = self.modclient.read_input_Registers(8198,1)
        return input_registers    

    def open_project_state(self):
        """
            (0) Idle, (1) Opening, (2) Failed
        """
        input_registers = self.modclient.read_input_Registers(8204,1)
        return input_registers    

    def task_state(self, task_num):
        """
        (0) Task idle
        (1) Task initialed
        (2) Task running
        (3) Task exit
        (4) Task pause
        (5) Task error
        """
        input_registers = self.modclient.read_input_Registers(8448+task_num,1)
        return input_registers
        
    def task_status(self):
        """
            RUN_READY : NRPL tasks are ready to run when bit turn on.
            RUNNING : NRPL programs (tasks) are running when bit turn on.
        """
        input_registers = self.modclient.read_input_Registers(8205,1)
        if input_registers[0] == True:
            status = "RUN_READY"
            return status
        if input_registers[1] == True:
            status = "RUNNING"
            return status
        else:
            return "0"

    def system_status(self):
        """
            ERROR : System is in ERROR state when bit turn on. (Safety state=ERROR)
            ENABLE : System is in ENABLE state when bit turn on. (Safety state=ENABLE or RUN)
            TASK_INIT : NRPL tasks are at program entry point when bit turn on.
            IDLE : System is in IDLE state.(Safety state!=RUN && != ERROR)
        """
        input_registers = self.modclient.read_input_Registers(8205,1)
        if input_registers[3] == True:
            status = "ERROR"
            return status
        if input_registers[4] == True:
            status = "ENABLE"
            return status
        if input_registers[5] == True:
            status = "TASK_INIT"
            return status
        if input_registers[6] == True:
            status = "IDLE"
            return status
        else:
            return "0"

    def task_run_status(self,task_num):
        """
            Task#n is running.
        """
        input_registers = self.modclient.read_input_Registers(8206,task_num)
        return input_registers