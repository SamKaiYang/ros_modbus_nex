#!/usr/bin/env python
import rospy
from modbus.modbus_wrapper_client import ModbusWrapperClient 
from std_msgs.msg import Int32MultiArray as HoldingRegister

NUM_REGISTERS = 1
ADDRESS_WRITE_START = 4096

if __name__=="__main__":
    rospy.init_node("modbus_client")
    
    host = "192.168.0.6"
    port = 502

    # setup modbus client    
    modclient = ModbusWrapperClient(host,port=port,rate=50,reset_registers=False,sub_topic="modbus_wrapper/output",pub_topic="modbus_wrapper/input")
    modclient.setWritingRegisters(ADDRESS_WRITE_START,NUM_REGISTERS)
    rospy.loginfo("Setup complete")
    
    register = 4096
    value = 0
    timeout = 0
    modclient.setOutput(register,value,timeout)
    rospy.loginfo("Set and individual output")