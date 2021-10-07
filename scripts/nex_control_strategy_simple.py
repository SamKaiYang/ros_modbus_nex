#!/usr/bin/env python
import rospy
from modbus.modbus_nex_api import ModbusNexApi 
from std_msgs.msg import Int32MultiArray as HoldingRegister

Stop_motion_flag = False
Start_motion_flag = False

if __name__=="__main__":
    rospy.init_node("control_strategy")
    nex_api = ModbusNexApi()
    rospy.loginfo("API setting")
    
## Test all program run, can get task state
    if nex_api.operation_mode_state() == "EXT":
        nex_api.send_reset(4096)
        nex_api.reload_all_programs()
        nex_api.enable_robot()

        # if nex_api.is_idle == True:
        nex_api.start_programs()
        while not rospy.is_shutdown():
            try:
                if Stop_motion_flag == False:
                    if nex_api.task_state(0) == "Task exit": #if Task exit
                        Stop_motion_flag = True
                else:
                    rospy.loginfo("Stop programs")
                    nex_api.stop_programs()
                    nex_api.disable_robot()
                    break
            except Exception, e:
                Stop_motion_flag = False
                rospy.logwarn("Could not running. %s", str(e))
                raise e
        # else:
        #     Stop_motion_flag = False
        #     rospy.loginfo("The robot is moving or not enable, please send the command again")
    else:
        Stop_motion_flag = False
        rospy.loginfo("Please switch to external control mode")

## Test read input register test 
    
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
