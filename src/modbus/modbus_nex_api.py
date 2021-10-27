#!/usr/bin/env python
import rospy
from modbus.modbus_wrapper_client import ModbusWrapperClient 
from std_msgs.msg import Int32MultiArray as HoldingRegister
import time

WRITE_NUM_REGISTERS = 36
READ_INPUT_NUM_REGISTERS = 272
ADDRESS_WRITE_START = 4096
ADDRESS_READ_INPUT_START = 8192

class PCS_actual():
    def __init__(self):
        self.X = None
        self.Y = None
        self.Z = None
        self.A = None
        self.B = None
        self.C = None

class PCS_command():
    def __init__(self):
        self.X = None
        self.Y = None
        self.Z = None
        self.A = None
        self.B = None
        self.C = None

class ACS_actual():
    def __init__(self):
        self.axis1 = None
        self.axis2 = None
        self.axis3 = None
        self.axis4 = None
        self.axis5 = None
        self.axis6 = None

class ACS_command():
    def __init__(self):
        self.axis1 = None
        self.axis2 = None
        self.axis3 = None
        self.axis4 = None
        self.axis5 = None
        self.axis6 = None

class ModbusNexApi():
    def __init__(self):
        self.pcs_actual = PCS_actual()
        self.pcs_command = PCS_command()
        self.acs_actual = ACS_actual()
        self.acs_command = ACS_command()
        self.host = "192.168.0.6" # 192.168.0.6
        self.port = 502
        self.rate = 50
        self.reset_registers = False
        self.modclient = ModbusWrapperClient(self.host, self.port, self.rate, self.reset_registers)

        self.modclient.setWritingRegisters(ADDRESS_WRITE_START,WRITE_NUM_REGISTERS)
        # self.modclient.setReadingRegisters(ADDRESS_READ_START,READ_NUM_REGISTERS)
        self.modclient.setReadingInputRegisters(ADDRESS_READ_INPUT_START,READ_INPUT_NUM_REGISTERS)
        rospy.loginfo("Setup complete")

    def ip_set(self, ip):
        self.host = ip 
        self.port = 502
        self.rate = 50
        self.reset_registers = False
        self.modclient = ModbusWrapperClient(self.host, self.port, self.rate, self.reset_registers)

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
            return byte & ~(1 << index)
    
    # ----------------Write msg to modbus server API-------------------
    def send_reset(self, address):
        self.modclient.setOutput(address,0,0)
        rospy.sleep(0.5)
        rospy.loginfo("RESET")

    def send_reset_other_state(self, address, output):
        self.modclient.setOutput(address,output,0)
        rospy.sleep(0.5)
        rospy.loginfo("RESET")

    def start_programs(self, num):
        """
            Starting loaded programs (rising edge)
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),0,1)
        register = 4096
        self.modclient.setOutput(register,value,0)
        rospy.sleep(0.5)
        if self.task_state(num) == "Task running":
            rospy.loginfo("START")
            return True
        else:
            rospy.loginfo("CAN'T START")
            return False

    # TODO: when start programs is set, this function is only available
    def stop_programs(self):
        """
            Stopped running programs (falling edge)
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),0,0)
        register = 4096
        self.modclient.setOutput(register,value,0)
        rospy.sleep(0.5)
        rospy.loginfo("STOP")

    def reset_error_robot(self):
        """
            reset robot
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),1,1)
        register = 4096
        self.modclient.setOutput(register,value,0)
        rospy.sleep(0.5)
        rospy.loginfo("Reset error")
        
    def enable_robot(self):
        """
            Enable robot (rising edge)
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),2,1)
        register = 4096
        self.modclient.setOutput(register,value,0)
        rospy.sleep(0.5)
        rospy.loginfo("ENABLE")

    # TODO: when programs is stop or not use, this function is only available
    def disable_robot(self):
        """
            Disable robot (falling edge)
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),2,0)
        register = 4096
        self.modclient.setOutput(register,value,0)
        rospy.sleep(0.5)
        rospy.loginfo("DISABLE")
        
    def reload_all_programs(self):
        """
            Reload all programs (*2)
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),3,1)
        register = 4096
        self.modclient.setOutput(register,value,0)
        rospy.sleep(0.5)
        rospy.loginfo("RELOAD_ALL")

    def reload_sel_programs(self):
        """
            Reload selected programs according to register:1001h (*3)
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),4,1)
        register = 4096
        self.modclient.setOutput(register,value,0)
        rospy.sleep(0.5)
        rospy.loginfo("RELOAD_SEL")
        
    def shutdown_controller(self):
        """
            Shutdown controller
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),10,1)
        register = 4096
        self.modclient.setOutput(register,value,0)
        rospy.sleep(0.5)
        rospy.loginfo("SHUTDOWN")

    # TODO: add select project name function 
    def project_name(self, name_str):
        """
            Specfied a project name to be opened. (*4)
            Using ASCII code (C-style string).
            Maximum 64 characters including terminating null-character.
        """
        project_name_str = name_str + "\0"
        str_to_bin = bin(int(binascii.hexlify(project_name_str), 16))
        bin_to_int = int(str_to_bin,2)
        register = 4096
        self.modclient.setOutput(register,bin_to_int,0)
        rospy.sleep(0.5)
        rospy.loginfo("project name set")

    def open_project(self):
        """
            Open selected project according to 1004h~1035h  (*4)
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),8,1)
        register = 4096
        self.modclient.setOutput(register,value,0)
        rospy.sleep(0.5)
        rospy.loginfo("open project")

    # ----------------Request modbus server to read server state API-------------------
    def operation_mode_state(self):
        """
            (0)T1, (1)T2, (2)AUT, (3)EXT
        """
        input_registers = self.modclient.read_input_Registers(8192,1)
        _bin = bin(int(input_registers[0]))[2:]
        # print("bin:",_bin)
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
        # print("bin:",_bin)
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
        # print("bin:",_bin)
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
        # print("bin:",_bin)
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
        # # print("bin:",_bin)
        
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
        bit_val = self.get_bit_val(input_registers[0],0)
        _bin = bin(int(bit_val))[2:]
        # # print("bin:",_bin)

        if _bin == "1":
            return True
        else:
            return False

    def is_running(self):
        """
            NRPL programs (tasks) are running when bit turn on.
        """
        input_registers = self.modclient.read_input_Registers(8205,1)
        bit_val = self.get_bit_val(input_registers[0],1)
        _bin = bin(int(bit_val))[2:]
        # # print("bin:",_bin)

        if _bin == "1":
            return True
        else:
            return False

    def is_in_ext_mode(self):
        """
            System is in EXT operation mode when bit turn on.
        """
        input_registers = self.modclient.read_input_Registers(8205,1)
        bit_val = self.get_bit_val(input_registers[0],2)
        _bin = bin(int(bit_val))[2:]
        # print("bin:",_bin)

        if _bin == "1":
            return True
        else:
            return False

    def is_error(self):
        """
            System is in ERROR state when bit turn on. (Safety state=ERROR)
        """
        input_registers = self.modclient.read_input_Registers(8205,1)
        bit_val = self.get_bit_val(input_registers[0],3)
        _bin = bin(int(bit_val))[2:]
        # print("bin:",_bin)

        if _bin == "1":
            return True
        else:
            return False

    def is_enable(self):
        """
            System is in ENABLE state when bit turn on. (Safety state=ENABLE or RUN)
        """
        input_registers = self.modclient.read_input_Registers(8205,1)
        bit_val = self.get_bit_val(input_registers[0],4)
        _bin = bin(int(bit_val))[2:]
        # print("bin:",_bin)

        if _bin == "1":
            return True
        else:
            return False

    def is_task_init(self):
        """
            NRPL tasks are at program entry point when bit turn on.
        """
        input_registers = self.modclient.read_input_Registers(8205,1)
        bit_val = self.get_bit_val(input_registers[0],5)
        _bin = bin(int(bit_val))[2:]
        # print("bin:",_bin)

        if _bin == "1":
            return True
        else:
            return False

    def is_idle(self):
        """
            System is in IDLE state.(Safety state!=RUN && != ERROR)
        """
        input_registers = self.modclient.read_input_Registers(8205,1)
        bit_val = self.get_bit_val(input_registers[0],6)
        _bin = bin(int(bit_val))[2:]
        # print("bin:",_bin)

        if _bin == "1":
            return True
        else:
            return False


    # TODO: add read input register pcs & angle
    def read_PCS_actual_position(self):
        """
            read_PCS_actual_position (double)
        """
        self.pcs_actual.X = self.modclient.read_input_Registers(16528,4)
        self.pcs_actual.Y = self.modclient.read_input_Registers(16532,4)
        self.pcs_actual.Z = self.modclient.read_input_Registers(16536,4)
        self.pcs_actual.A = self.modclient.read_input_Registers(16540,4)
        self.pcs_actual.B = self.modclient.read_input_Registers(16544,4)
        self.pcs_actual.C = self.modclient.read_input_Registers(16548,4)
        return self.pcs_actual

    def read_PCS_command_position(self):
        """
            read_PCS_command_position (double)
        """
        self.pcs_command.X = self.modclient.read_input_Registers(16464,4)
        self.pcs_command.Y = self.modclient.read_input_Registers(16468,4)
        self.pcs_command.C = self.modclient.read_input_Registers(16472,4)
        self.pcs_command.A = self.modclient.read_input_Registers(16476,4)
        self.pcs_command.B = self.modclient.read_input_Registers(16480,4)
        self.pcs_command.C = self.modclient.read_input_Registers(16484,4)
        return self.pcs_command

    def read_ACS_actual_position(self):
        """
            read_ACS_actual_position (double)
        """
        self.acs_actual.axis1 = self.modclient.read_input_Registers(16432,4)
        self.acs_actual.axis2 = self.modclient.read_input_Registers(16436,4)
        self.acs_actual.axis3 = self.modclient.read_input_Registers(16440,4)
        self.acs_actual.axis4 = self.modclient.read_input_Registers(16444,4)
        self.acs_actual.axis5 = self.modclient.read_input_Registers(16448,4)
        self.acs_actual.axis6 = self.modclient.read_input_Registers(16452,4)
        return self.acs_actual

    def read_ACS_command_position(self):
        """
            read_ACS_command_position (double)
        """
        self.acs_command.axis1 = self.modclient.read_input_Registers(16400,4)
        self.acs_command.axis2 = self.modclient.read_input_Registers(16404,4)
        self.acs_command.axis3 = self.modclient.read_input_Registers(16408,4)
        self.acs_command.axis4 = self.modclient.read_input_Registers(16412,4)
        self.acs_command.axis5 = self.modclient.read_input_Registers(16416,4)
        self.acs_command.axis6 = self.modclient.read_input_Registers(16420,4)
        return self.acs_command
        