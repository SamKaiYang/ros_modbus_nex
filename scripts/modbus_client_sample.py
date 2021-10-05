#!/usr/bin/env python
import rospy
from modbus.modbus_wrapper_client import ModbusWrapperClient 
from std_msgs.msg import Int32MultiArray as HoldingRegister
import time

WRITE_NUM_REGISTERS = 36
READ_NUM_REGISTERS = 272
ADDRESS_WRITE_START = 4096
ADDRESS_READ_START = 8192

if __name__=="__main__":
    rospy.init_node("modbus_client")
    
    host = "192.168.0.6"
    port = 502

    # setup modbus client    
    modclient = ModbusWrapperClient(host,port=port,rate=50,reset_registers=False,sub_topic="modbus_wrapper/output",pub_topic="modbus_wrapper/input")
    modclient.setReadingRegisters(ADDRESS_READ_START,READ_NUM_REGISTERS)
    modclient.setWritingRegisters(ADDRESS_WRITE_START,WRITE_NUM_REGISTERS)
    rospy.loginfo("Setup complete")

# ----  test read register
    
    # Operation mode
    # 8192
    # (0)T1, (1)T2, (2)AUT, (3)EXT

    # Safety state
    # 8193
    # (0)Disable, (1)Ready, (2)Error, (3)Enable, (4)Running

    # Enable switch state
    # 8198
    # (0)Disable,(1)Enable, (2)Pressed-EMG


    # Task#0 state
    # 8448
    # (0) Task idle
    # (1) Task initialed
    # (2) Task running
    # (3) Task exit
    # (4) Task paused
    # (5) Task error
    while not rospy.is_shutdown():
        try: 
            Operation_mode = modclient.readRegisters(8192,272)
            Safety_state = modclient.readRegisters(8193,272)
            Enable_switch_state = modclient.readRegisters(8198,272)
            Task0_state = modclient.readRegisters(8448,272)

            rospy.loginfo("Operation mode:%d" % Operation_mode)
            rospy.loginfo("Safety state:%d" % Safety_state)
            rospy.loginfo("Enable switch state:%d" % Enable_switch_state)
            rospy.loginfo("Task#0 state:%d" % Task0_state)
        except Exception,e:
            rospy.logwarn("Could not read. %s", str(e))
            raise e
            rospy.sleep(2)
    time.sleep(2)
# ----  sample 1 cmd test
    '''
    register = 4096
    value = 0
    timeout = 0
    modclient.setOutput(register,value,timeout)
    rospy.loginfo("Set and individual output")
    '''
# ---- sample 1 loop test
    ''' 
    register = 4096
    # value = 0
    timeout = 0
    # modclient.setOutput(register,value,timeout)
    # rospy.loginfo("Set and individual output")

    modclient.setOutput(register,8,timeout)
    time.sleep(0.5)
    modclient.setOutput(register,4,timeout)
    time.sleep(0.5)
    modclient.setOutput(register,5,timeout)
    time.sleep(30)
    modclient.setOutput(register,4,timeout)
    time.sleep(0.5)
    modclient.setOutput(register,0,timeout)
    time.sleep(0.5)
    '''
# ---- nex modbus externel control api list 
    
    # Reload 8-> Enable 4-> START 5 ->STOP & disable 0 or STOP 4 -> disable 0
    
    # # Enable robot (rising edge)
    # register = 4096
    # value = 4
    # rospy.loginfo("ENABLE")

    # # Reload all programs (*2)
    # register = 4096
    # value = 8   
    # rospy.loginfo("RELOAD_ALL")

    # # Starting loaded programs (rising edge)
    # register = 4096
    # value = 5 # 4+1
    # rospy.loginfo("START")

    # # Stopped running programs (falling edge)
    # register = 4096
    # value = 0 // 4 + 0
    # rospy.loginfo("STOP")

    # # Shutdown controller
    # register = 4096
    # value = 1024
    # rospy.loginfo("SHUTDOWN")
    