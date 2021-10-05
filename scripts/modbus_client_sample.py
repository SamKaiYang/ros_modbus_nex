#!/usr/bin/env python
import rospy
from modbus.modbus_wrapper_client import ModbusWrapperClient 
from std_msgs.msg import Int32MultiArray as HoldingRegister
import time

WRITE_NUM_REGISTERS = 36
READ_NUM_REGISTERS = 10
ADDRESS_WRITE_START = 4096
ADDRESS_READ_START = 48193

def showUpdatedRegisters(msg):
    rospy.loginfo("Modbus server registers have been updated: %s",str(msg.data))

if __name__=="__main__":
    rospy.init_node("modbus_client")
    
    host = "192.168.0.6"
    port = 502

    # setup modbus client    
    modclient = ModbusWrapperClient(host,port=port,rate=50,reset_registers=False,sub_topic="modbus_wrapper/output",pub_topic="modbus_wrapper/input")
    modclient.setReadingRegisters(ADDRESS_READ_START,READ_NUM_REGISTERS)
    modclient.setWritingRegisters(ADDRESS_WRITE_START,WRITE_NUM_REGISTERS)
    start,num_registers = modclient.getReadingRegisters()
    rospy.loginfo("start:%d" % start)
    rospy.loginfo("num_registers:%d" % num_registers)
    rospy.loginfo("Setup complete")

    # start listening to modbus and publish changes to the rostopic
    # modclient.startListening()
    # rospy.loginfo("Listener started")
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
    
    #################
    # Example 2
    # Create a listener that show us a message if anything on the readable modbus registers change
    # rospy.loginfo("All done. Listening to inputs... Terminate by Ctrl+c")
    # sub = rospy.Subscriber("modbus_wrapper/input",HoldingRegister,showUpdatedRegisters,queue_size=500)
    #################

# ---- sample read input registers
    while not rospy.is_shutdown():
        try:
            input_registers = modclient.read_input_Registers(8193,1)
            print(input_registers)
            rospy.sleep(2)
        except Exception, e:
            rospy.logwarn("Could not read. %s", str(e))
            raise e
    
# ----  sample 1 cmd test
    
    # register = 4096
    # value = 0
    # timeout = 0
    # modclient.setOutput(register,value,timeout)
    # rospy.loginfo("Set and individual output")
    
# ---- sample 1 loop test
    
    # register = 4096
    # # value = 0
    # timeout = 0.5
    # # modclient.setOutput(register,value,timeout)
    # # rospy.loginfo("Set and individual output")

    # modclient.setOutput(register,8,timeout)
    # rospy.loginfo("Reload")
    # modclient.setOutput(register,4,timeout)
    # rospy.loginfo("Enable")
    # modclient.setOutput(register,5,30)
    # rospy.loginfo("START")
    # modclient.setOutput(register,4,timeout)
    # rospy.loginfo("STOP")
    # modclient.setOutput(register,0,timeout)
    # rospy.loginfo("disable")
    # modclient.stopListening()
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
    