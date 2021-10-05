#!/usr/bin/env python
import rospy
from modbus.modbus_wrapper_client import ModbusWrapperClient 
from std_msgs.msg import Int32MultiArray as HoldingRegister
from pynput import keyboard

WRITE_NUM_REGISTERS = 36
READ_NUM_REGISTERS = 272
ADDRESS_WRITE_START = 4096
ADDRESS_READ_START = 8192

Stop_motion_flag = False

if __name__=="__main__":
    rospy.init_node("modbus_client")
    
    host = "192.168.0.6"
    port = 502
    if rospy.has_param("~ip"):
        host =  rospy.get_param("~ip")
    else:
        rospy.loginfo("For not using the default IP %s, add an arg e.g.: '_ip:=\"192.168.0.199\"'",host)
    if rospy.has_param("~port"):
        port =  rospy.get_param("~port")
    else:
        rospy.loginfo("For not using the default port %d, add an arg e.g.: '_port:=1234'",port)
    
    # setup modbus client    
    modclient = ModbusWrapperClient(host,port=port,rate=50,reset_registers=False,sub_topic="modbus_wrapper/output",pub_topic="modbus_wrapper/input")
    modclient.setReadingRegisters(ADDRESS_READ_START,READ_NUM_REGISTERS)
    modclient.setWritingRegisters(ADDRESS_WRITE_START,WRITE_NUM_REGISTERS)
    rospy.loginfo("Setup complete")
    
    # TODO: Get register value check subscribe
    # start listening to modbus and publish changes to the rostopic
    modclient.startListening()
    rospy.loginfo("Listener started")


    def on_press(key):
        try:
            print('Alphanumeric key pressed: {0} '.format(
                key.char))
        except AttributeError:
            print('special key pressed: {0}'.format(
                key))

    def on_release(key):
        print('Key released: {0}'.format(
            key))
        print('Key released: {0}'.format(
            key.char))
        if key == keyboard.Key.esc:
            # Stop listener
            return False

        if key.char == "s":
            # Stop listener
            print("fuckkkk")
            Stop_motion_flag = True
            return False

    # Collect events until released
    with keyboard.Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        listener.join()
    
    # modclient.stopListening()
    while not rospy.is_shutdown():
        try: 
            if not rospy.is_shutdown() and modclient.stop_listener is True:
                # ---- test 
                register = 4096
                value = 4
                timeout = 0
                modclient.setOutput(register,value,timeout)
                rospy.loginfo("ENABLE")

                register = 4096
                value = 8
                timeout = 0
                modclient.setOutput(register,value,timeout)
                rospy.loginfo("RELOAD_ALL")

                register = 4096
                value = 0
                timeout = 0
                modclient.setOutput(register,value,timeout)
                rospy.loginfo("START")

                
            elif Stop_motion_flag == True:
                Stop_motion_flag = False
                register = 4096
                value = 1
                timeout = 0
                modclient.setOutput(register,value,timeout)
                rospy.loginfo("STOP")
                modclient.stopListening()
            elif modclient.stop_listener is False:
                modclient.closeConnection()

        except Exception,e:
            rospy.logwarn("Could not running. %s", str(e))
            raise e
            rospy.sleep(2)

    