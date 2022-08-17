#!/usr/bin/env python
import rospy
from modbus.modbus_wrapper_client import ModbusWrapperClient 
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.compat import iteritems
from collections import OrderedDict
from std_msgs.msg import Int32MultiArray as HoldingRegister
import time
import struct

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

# class ModbusNexApi():
class ModbusNexApi(object):
    def __init__(self):
        self.host = "192.168.0.6" # 192.168.0.6
        self.port = 502
        self.rate = 50
        self.reset_registers = False
        self.modclient = ModbusWrapperClient(self.host, self.port, self.rate, self.reset_registers)

        self.modclient.setWritingRegisters(ADDRESS_WRITE_START,WRITE_NUM_REGISTERS)
        # self.modclient.setReadingRegisters(ADDRESS_READ_START,READ_NUM_REGISTERS)
        self.modclient.setReadingInputRegisters(ADDRESS_READ_INPUT_START,READ_INPUT_NUM_REGISTERS)

    def ip_init(self):
        self.host = "192.168.0.6" # 192.168.0.6
        self.port = 502
        self.rate = 50
        self.reset_registers = False
        self.modclient = ModbusWrapperClient(self.host, self.port, self.rate, self.reset_registers)

        self.modclient.setWritingRegisters(ADDRESS_WRITE_START,WRITE_NUM_REGISTERS)
        # self.modclient.setReadingRegisters(ADDRESS_READ_START,READ_NUM_REGISTERS)
        self.modclient.setReadingInputRegisters(ADDRESS_READ_INPUT_START,READ_INPUT_NUM_REGISTERS)
        rospy.loginfo("Setup complete")
        
    def point_init(self):
        self.pcs_actual = PCS_actual()
        self.pcs_command = PCS_command()
        self.acs_actual = ACS_actual()
        self.acs_command = ACS_command()

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
        # rospy.sleep(0.2)
        rospy.loginfo("SET RESET")

    def send_reset_other_state(self, address, output):
        self.modclient.setOutput(address,output,0)
        # rospy.sleep(1)
        rospy.loginfo("RESET")

    def start_programs(self, num):
        """
            Starting loaded programs (rising edge)
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),0,1)
        register = 4096
        self.modclient.setOutput(register,value,0)
        # rospy.sleep(0.2)
        if self.task_state(num) == "Task running":
            rospy.loginfo("START")
            return True
        else:
            rospy.loginfo("CAN'T START")
            return False

    def stop_programs(self):
        """
            Stopped running programs (falling edge)
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),0,0)
        register = 4096
        self.modclient.setOutput(register,value,0)
        # rospy.sleep(0.2)
        rospy.loginfo("STOP")

    def reset_error_robot(self):
        """
            reset robot
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),1,1)
        register = 4096
        self.modclient.setOutput(register,value,0)
        # rospy.sleep(0.2)
        rospy.loginfo("Reset error")
        
    def enable_robot(self):
        """
            Enable robot (rising edge)
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),2,1)
        register = 4096
        self.modclient.setOutput(register,value,0)
        # rospy.sleep(1)
        rospy.loginfo("ENABLE")

    def disable_robot(self):
        """
            Disable robot (falling edge)
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),2,0)
        register = 4096
        self.modclient.setOutput(register,value,0)
        # rospy.sleep(0.2)
        rospy.loginfo("DISABLE")
        # self.send_reset(4096)
        # self.reset_error_robot()
        # self.send_reset(4096)
        
    def reload_all_programs(self):
        """
            Reload all programs (*2)
        """
        rospy.sleep(0.2)
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),3,1)
        register = 4096
        self.modclient.setOutput(register,value,0)
        rospy.sleep(0.2)
        rospy.loginfo("RELOAD ALL")

    def reload_sel_programs(self):
        """
            Reload selected programs according to register:1001h (*3)
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),4,1)
        register = 4096
        self.modclient.setOutput(register,value,0)
        # rospy.sleep(0.2)
        rospy.loginfo("RELOAD SEL")
        
    def shutdown_controller(self):
        """
            Shutdown controller
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),10,1)
        register = 4096
        self.modclient.setOutput(register,value,0)
        # rospy.sleep(0.2)
        rospy.loginfo("SHUTDOWN")

    # TODO: add select project name function 
    def project_name(self, name_str):
        """
            Specfied a project name to be opened. (*4)
            Using ASCII code (C-style string).
            Maximum 64 characters including terminating null-character.
        """
        count = 32
        project_name_str = name_str + "\0"
        builder = BinaryPayloadBuilder()
        builder.add_string(project_name_str)
        payload = builder.to_registers()
        payload = builder.build()
        register = 4100

        for i in range(len(payload)):
            payload[i] = struct.pack('<h', int(float(payload[i])))[0]
        self.modclient.setOutput(register,payload,0)
        # rospy.sleep(0.2)
        rospy.loginfo("project name set")

    def open_project(self):
        """
            Open selected project according to 1004h~1035h  (*4)
        """
        input_registers = self.modclient.readRegisters(4096,1)
        value = self.set_bit_val(int(input_registers[0]),8,1)
        register = 4096
        self.modclient.setOutput(register,value,0)
        # rospy.sleep(0.2)
        rospy.loginfo("open project")

    def set_pcs_position(self, x, y, z, a, b, c):
        """
            set TPUI SMO 64 start 111 end
        """
        builder = BinaryPayloadBuilder(byteorder= Endian.Big, wordorder=Endian.Little)
        builder.add_64bit_float(x)
        X = builder.to_registers()
        # X = builder.build()
        register = 1024 + 32 # 64
        self.modclient.setOutput(register,X,0)
        builder = BinaryPayloadBuilder(byteorder= Endian.Big, wordorder=Endian.Little)
        builder.add_64bit_float(y)
        Y = builder.to_registers()
        # Y = builder.build()
        register = 1024 + 36 # 72
        self.modclient.setOutput(register,Y,0)
        builder = BinaryPayloadBuilder(byteorder= Endian.Big, wordorder=Endian.Little)
        builder.add_64bit_float(z)
        Z = builder.to_registers()
        # Z = builder.build()
        register = 1024 + 40 # 80
        self.modclient.setOutput(register,Z,0)
        builder = BinaryPayloadBuilder(byteorder= Endian.Big, wordorder=Endian.Little)
        builder.add_64bit_float(a)
        A = builder.to_registers()
        # A = builder.build()
        register = 1024 + 44 # 88
        self.modclient.setOutput(register,A,0)
        builder = BinaryPayloadBuilder(byteorder= Endian.Big, wordorder=Endian.Little)
        builder.add_64bit_float(b)
        B = builder.to_registers()
        # B = builder.build()
        register = 1024 + 48 # 96
        self.modclient.setOutput(register,B,0)
        builder = BinaryPayloadBuilder(byteorder= Endian.Big, wordorder=Endian.Little)
        builder.add_64bit_float(c)
        C = builder.to_registers()
        # C = builder.build()
        register = 1024 + 52 # 104
        self.modclient.setOutput(register,C,0)

        # rospy.sleep(0.2)
        rospy.loginfo("set pcs position")

    def set_acs_position(self, axis1, axis2, axis3, axis4, axis5, axis6):
        """
            set TPUI SMO 112 start 159 end 
        """
        builder = BinaryPayloadBuilder(byteorder= Endian.Big, wordorder=Endian.Little)
        builder.add_64bit_float(axis1)
        Axis1 = builder.to_registers()
        # Axis1 = builder.build()
        register = 1024 + 56 # 112
        self.modclient.setOutput(register,Axis1,0)
        builder = BinaryPayloadBuilder(byteorder= Endian.Big, wordorder=Endian.Little)
        builder.add_64bit_float(axis2)
        Axis2 = builder.to_registers()
        # Axis2 = builder.build()
        register = 1024 + 60 # 120
        self.modclient.setOutput(register,Axis2,0)
        builder = BinaryPayloadBuilder(byteorder= Endian.Big, wordorder=Endian.Little)
        builder.add_64bit_float(axis3)
        Axis3 = builder.to_registers()
        # Axis3 = builder.build()
        register = 1024 + 64 # 128
        self.modclient.setOutput(register,Axis3,0)
        builder = BinaryPayloadBuilder(byteorder= Endian.Big, wordorder=Endian.Little)
        builder.add_64bit_float(axis4)
        Axis4 = builder.to_registers()
        # Axis4 = builder.build()
        register = 1024 + 68 # 136
        self.modclient.setOutput(register,Axis4,0)
        builder = BinaryPayloadBuilder(byteorder= Endian.Big, wordorder=Endian.Little)
        builder.add_64bit_float(axis5)
        Axis5 = builder.to_registers()
        # Axis5 = builder.build()
        register = 1024 + 72 # 144
        self.modclient.setOutput(register,Axis5,0)
        builder = BinaryPayloadBuilder(byteorder= Endian.Big, wordorder=Endian.Little)
        builder.add_64bit_float(axis6)
        Axis6 = builder.to_registers()
        # Axis6 = builder.build()
        register = 1024 + 76 # 152
        self.modclient.setOutput(register,Axis6,0)
        
        # rospy.sleep(0.2)
        rospy.loginfo("set acs position")

    def send_64bit_value(self, register, value):
        builder = BinaryPayloadBuilder(byteorder= Endian.Big, wordorder=Endian.Little)
        builder.add_64bit_float(value)
        Value = builder.to_registers()
        # Axis1 = builder.build()
        to_register = register
        self.modclient.setOutput(to_register,Value,0)
        # rospy.sleep(0.2)

    def send_16bit_value(self, register, value):
        builder = BinaryPayloadBuilder(byteorder= Endian.Big, wordorder=Endian.Little)
        builder.add_16bit_int(value)
        Value = builder.to_registers()
        # Axis1 = builder.build()
        to_register = register
        self.modclient.setOutput(to_register,Value,0)
        # rospy.sleep(0.2)
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

    def read_PCS_actual_position(self):
        """
            read_PCS_actual_position (double)
        """
        result = self.modclient.read_input_Registers(16528,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.pcs_actual.X = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16532,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.pcs_actual.Y = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16536,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.pcs_actual.Z = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16540,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.pcs_actual.A = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16544,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.pcs_actual.B = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16548,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.pcs_actual.C = decoder.decode_64bit_float()

        return self.pcs_actual

    def read_PCS_command_position(self):
        """
            read_PCS_command_position (double)
        """
        result = self.modclient.read_input_Registers(16464,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.pcs_command.X = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16468,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.pcs_command.Y = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16472,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.pcs_command.Z = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16476,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.pcs_command.A = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16480,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.pcs_command.B = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16484,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.pcs_command.C = decoder.decode_64bit_float()
        return self.pcs_command

    def read_ACS_actual_position(self):
        """
            read_ACS_actual_position (double)
        """
        result = self.modclient.read_input_Registers(16432,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.acs_actual.axis1 = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16436,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.acs_actual.axis2 = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16440,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.acs_actual.axis3 = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16444,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.acs_actual.axis4 = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16448,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.acs_actual.axis5 = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16452,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.acs_actual.axis6 = decoder.decode_64bit_float()

        return self.acs_actual

    def read_ACS_command_position(self):
        """
            read_ACS_command_position (double)
        """
        result = self.modclient.read_input_Registers(16400,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.acs_command.axis1 = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16404,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.acs_command.axis2 = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16408,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.acs_command.axis3 = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16412,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.acs_command.axis4 = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16416,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.acs_command.axis5 = decoder.decode_64bit_float()

        result = self.modclient.read_input_Registers(16420,4)
        decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder= Endian.Big, wordorder=Endian.Little)
        self.acs_command.axis6 = decoder.decode_64bit_float()
        return self.acs_command
        
    def read_project_name(self):
        """
            Maximum 64 char, C-String, include null terminating char.
        """
        count = 32 #Read 32 16bit registers
        result = self.modclient.read_input_Registers(8208,count)
        for i in range(len(result)):
            result[i] = struct.unpack("<H", struct.pack(">H", result[i]))[0]

        decoder = BinaryPayloadDecoder.fromRegisters(result)
        project_name = decoder.decode_string(len(result))#Since string is 64 characters long
        # print(project_name)
        return project_name