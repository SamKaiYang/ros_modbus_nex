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
            input_registers = nex_api.operation_mode_state()
            print("operation_mode_state:",input_registers)
            input_registers = nex_api.safety_state()
            print("safety_state:",input_registers)
            input_registers = nex_api.enable_switch_state()
            print("enable_switch_state:",input_registers)
            input_registers = nex_api.open_project_state()
            print("open_project_state:",input_registers)
            input_registers = nex_api.task_state(0)
            print("task_state:",input_registers)
            input_registers = nex_api.task_status()
            print("task_status:",input_registers)
            input_registers = nex_api.system_status()
            print("system_status:",input_registers)
            # input_registers = nex_api.task_run_status(0)
            # print("task_run_status:",input_registers)

            print("\n\n")
            rospy.sleep(2)
            
        except Exception, e:
            rospy.logwarn("Could not read any callback information. %s", str(e))
            raise e
            rospy.sleep(1)