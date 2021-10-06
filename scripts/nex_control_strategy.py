#!/usr/bin/env python
import rospy
from modbus.modbus_nex_api import ModbusNexApi 
from std_msgs.msg import Int32MultiArray as HoldingRegister
from pynput import keyboard

Stop_motion_flag = False
Start_motion_flag = False
def on_press(key):
    try:
        print('Alphanumeric key pressed: {0} '.format(
            key.char))
    except AttributeError:
        print('special key pressed: {0}'.format(
            key))

def on_release(key):
    global Stop_motion_flag, Start_motion_flag
    print('Key released: {0}'.format(
        key))
    print('Key released: {0}'.format(
        key.char))
    if key == keyboard.Key.esc:
        return False

    if key.char == "s":
        print("start flag enable")
        Start_motion_flag = True
        return False

    if key.char == "e":
        print("stop flag enable")
        Stop_motion_flag = True
        return False

if __name__=="__main__":
    rospy.init_node("control_strategy")
    nex_api = ModbusNexApi()
    rospy.loginfo("API setting")

    # nex_api.reload_all_programs()
    # nex_api.enable_robot()
    # nex_api.start_programs()


    # Stop_motion_flag == True

    # while not rospy.is_shutdown():
    #     try:
    #         if Stop_motion_flag == True:
    #             Stop_motion_flag = False
    #             rospy.loginfo("Stop programs")
    #             nex_api.stop_programs()
    #             nex_api.disable_robot()
    #         else:
    #             pass

    #     except Exception,e:
    #         rospy.logwarn("Could not running. %s", str(e))
    #         raise e
    #         rospy.sleep(2)

    # read input register test 

    while not rospy.is_shutdown():
        try:
            pass
        except Exception, e:
            rospy.logwarn("Could not read any callback information. %s", str(e))
            raise e
            rospy.sleep(1)