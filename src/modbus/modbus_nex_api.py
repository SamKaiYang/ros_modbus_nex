#!/usr/bin/env python
import rospy
from modbus.modbus_wrapper_client import ModbusWrapperClient 

class ModbusNexApi():
    def __init__(self):
        self.__rate = rate
        self.__reading_delay = 1/rate
        self.post = Post(self)
        
        self.__reset_registers = reset_registers
        self.__reading_register_start = 0
        self.__num_reading_registers = 20
#         self.input_size = 16
        self.__input = HoldingRegister()
        self.__input.data = [0 for i in xrange(self.__num_reading_registers )]

        self.__writing_registers_start = ADDRESS_WRITE_START
        self.__num_writing_registers =  20
#         self.output_size = 16 
        self.__output = [None for i in range(self.__num_writing_registers)]
        
        self.__last_output_time = rospy.get_time()
        self.__mutex = Lock()
        
        self.__sub = rospy.Subscriber(sub_topic,HoldingRegister,self.__updateModbusOutput,queue_size=500)
        self.__pub = rospy.Publisher(pub_topic,HoldingRegister,queue_size=500, latch=True)
        
        rospy.on_shutdown(self.closeConnection)
    
    def startListening(self):
        """
            Non blocking call for starting the listener for the readable modbus server registers 
        """
        #start reading the modbus
        self.post.__updateModbusInput()