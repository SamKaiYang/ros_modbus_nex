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

    def get_bit_val(self, byte, index):
        """
        :param byte: the byte value of the value to be retrieved

        :param index: the serial number of the bit to be read, starting from right to left 0, 0-7 are 8 bits of a complete byte

        :returns: Returns the value of this bit, 0 or 1
        """
        if byte & (1 << index):
            return 1
        else:
            return 0

    def set_bit_val(self, byte, index, val):
        """
        :param byte: the original value of the byte to be changed

        :param index: the serial number of the bit to be changed, starting from right to left 0, 0-7 are 8 bits of a complete byte

        :param val: The value of the target bit to be changed beforehand, 0 or 1

        :returns: returns the value of the changed byte
        """
        if val:
            return byte | (1 << index)
        else:
            return byte % ~(1 << index)
    
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
        _bin = bin(int(input_registers[0]))[2:]
        print("bin:",_bin)
        if _bin == "0":
            return "T1"
        elif _bin == "1":
            return "T2"
        elif _bin == "10":
            return "AUT"
        elif _bin == "11":
            return "EXT"
        else:
            return "ERROR"

    def safety_state(self):
        """
            (0)Disable, (1)Ready, (2)Error, (3)Enable, (4)Running
        """
        input_registers = self.modclient.read_input_Registers(8193,1)
        _bin = bin(int(input_registers[0]))[2:]
        print("bin:",_bin)
        if _bin == "0":
            return "Disable"
        elif _bin == "1":
            return "Ready"
        elif _bin == "10":
            return "Error"
        elif _bin == "11":
            return "Enable"
        elif _bin == "100":
            return "Running"
        else:
            return "ERROR"

    def enable_switch_state(self):
        """
            (0)Disable,(1)Enable, (2)Pressed-EMG
        """
        input_registers = self.modclient.read_input_Registers(8198,1)
        _bin = bin(int(input_registers[0]))[2:]
        print("bin:",_bin)
        if _bin == "0":
            return "Disable"
        elif _bin == "1":
            return "Enable"
        elif _bin == "10":
            return "Pressed-EMG"
        else:
            return "ERROR"

    def open_project_state(self):
        """
            (0) Idle, (1) Opening, (2) Failed
        """
        input_registers = self.modclient.read_input_Registers(8204,1)
        _bin = bin(int(input_registers[0]))[2:]
        print("bin:",_bin)
        if _bin == "0":
            return "Idle"
        elif _bin == "1":
            return "Opening"
        elif _bin == "10":
            return "Failed"
        else:
            return "ERROR"

    def task_state(self, task_num):
        """
        (0) Task idle
        (1) Task initialed
        (2) Task running
        (3) Task exit
        (4) Task pause
        (5) Task error
        """
        address = 8448+task_num
        input_registers = self.modclient.read_input_Registers(address,1)
        _bin = bin(int(input_registers[0]))[2:]
        print("bin:",_bin)
        
        if _bin == "0":
            return "Task idle"
        elif _bin == "1":
            return "Task initialed"
        elif _bin == "10":
            return "Task running"
        elif _bin == "11":
            return "Task exit"
        elif _bin == "100":
            return "Task pause"
        elif _bin == "101":
            return "Task error"
        else:
            return "ERROR"

    def is_run_ready(self):
        """
            NRPL tasks are ready to run when bit turn on.
        """
        input_registers = self.modclient.read_input_Registers(8205,1)
        _bin = bin(int(input_registers[0]))[2:]
        print("bin:",_bin)

        if self.get_bit_val(_bin,0) == 1:
            return True
        else:
            return False

    def is_running(self):
        """
            NRPL programs (tasks) are running when bit turn on.
        """
        input_registers = self.modclient.read_input_Registers(8205,1)
        _bin = bin(int(input_registers[0]))[2:]
        print("bin:",_bin)

        if self.get_bit_val(_bin,1) == 1:
            return True
        else:
            return False

    def is_in_ext_mode(self):
        """
            System is in EXT operation mode when bit turn on.
        """
        input_registers = self.modclient.read_input_Registers(8205,1)
        _bin = bin(int(input_registers[0]))[2:]
        print("bin:",_bin)

        if self.get_bit_val(_bin,2) == 1:
            return True
        else:
            return False

    def is_error(self):
        """
            System is in ERROR state when bit turn on. (Safety state=ERROR)
        """
        input_registers = self.modclient.read_input_Registers(8205,1)
        _bin = bin(int(input_registers[0]))[2:]
        print("bin:",_bin)

        if self.get_bit_val(_bin,3) == 1:
            return True
        else:
            return False

    def is_enable(self):
        """
            System is in ENABLE state when bit turn on. (Safety state=ENABLE or RUN)
        """
        input_registers = self.modclient.read_input_Registers(8205,1)
        _bin = bin(int(input_registers[0]))[2:]
        print("bin:",_bin)

        if self.get_bit_val(_bin,4) == 1:
            return True
        else:
            return False

    def is_task_init(self):
        """
            NRPL tasks are at program entry point when bit turn on.
        """
        input_registers = self.modclient.read_input_Registers(8205,1)
        _bin = bin(int(input_registers[0]))[2:]
        print("bin:",_bin)

        if self.get_bit_val(_bin,5) == 1:
            return True
        else:
            return False

    def is_idle(self):
        """
            System is in IDLE state.(Safety state!=RUN && != ERROR)
        """
        input_registers = self.modclient.read_input_Registers(8205,1)
        _bin = bin(int(input_registers[0]))[2:]
        print("bin:",_bin)

        if self.get_bit_val(_bin,6) == 1:
            return True
        else:
            return False

    # def task_run_status(self,task_num):
    #     """
    #         Task#n is running.
    #     """
    #     input_registers = self.modclient.read_input_Registers(8206,task_num)
    #     return input_registers[0]