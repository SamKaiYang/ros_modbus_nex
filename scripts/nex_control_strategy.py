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

    nex_api.reload_all_programs()
    nex_api.enable_robot()
    nex_api.start_programs()


    

    while not rospy.is_shutdown():
        try:
            if Stop_motion_flag == False:
                if nex_api.task_state(0) == "Task exit": #if Task exit
                    Stop_motion_flag = True
                    print("aaa")
            else:
                # Stop_motion_flag = False
                rospy.loginfo("Stop programs")
                nex_api.stop_programs()
                nex_api.disable_robot()
                break
        except Exception,e:
            rospy.logwarn("Could not running. %s", str(e))
            raise e
            rospy.sleep(2)

    # read input register test 
    
    # while not rospy.is_shutdown():
    #     input_registers = nex_api.operation_mode_state()
    #     print("operation_mode_state:",input_registers)
    #     print("\n")
    #     input_registers = nex_api.safety_state()
    #     print("safety_state:",input_registers)
    #     print("\n")
    #     input_registers = nex_api.enable_switch_state()
    #     print("enable_switch_state:",input_registers)
    #     print("\n")
    #     input_registers = nex_api.open_project_state()
    #     print("open_project_state:",input_registers)
    #     print("\n")
    #     input_registers = nex_api.task_state(0)
    #     print("task_state:",input_registers)
    #     print("\n")
    #     input_registers = nex_api.is_run_ready()
    #     print("is_run_ready:",input_registers)
    #     print("\n")
    #     input_registers = nex_api.is_running()
    #     print("is_running:",input_registers)
    #     print("\n")
    #     input_registers = nex_api.is_in_ext_mode()
    #     print("is_in_ext_mode:",input_registers)
    #     print("\n")
    #     input_registers = nex_api.is_error()
    #     print("is_error:",input_registers)
    #     print("\n")
    #     input_registers = nex_api.is_enable()
    #     print("is_enable:",input_registers)
    #     print("\n")
    #     input_registers = nex_api.is_task_init()
    #     print("is_task_init:",input_registers)
    #     print("\n")
    #     input_registers = nex_api.is_idle()
    #     print("is_idle:",input_registers)

    #     print("\n\n")
    #     rospy.sleep(2)
