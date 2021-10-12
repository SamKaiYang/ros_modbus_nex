#!/usr/bin/env python
import rospy
from modbus.modbus_nex_api import ModbusNexApi 
from std_msgs.msg import Int32MultiArray as HoldingRegister
from std_msgs.msg import String
from modbus.msg import arm_callback
from agv_test.msg import send

class nex_control:
    def __init__(self):
        self.task_cmd = 0
        self.nex_api = ModbusNexApi()
        rospy.loginfo("API setting")
        self.Stop_motion_flag = False
        self.Start_motion_flag = False
        self.pub_armstatus = rospy.Publisher("/arm_status",arm_callback,queue_size=10)
        self.sub_taskcmd = rospy.Subscriber("/armTask_cmd",send,self.callback)
    
    def callback(self,data):
        self.task_cmd = data.task_cmd
        rospy.loginfo("I heard armTask_cmd is %s", data.task_cmd)

    def arm_task_program(self):
        ## Test all program run, can get task state
        if self.nex_api.operation_mode_state() == "EXT":
            self.nex_api.send_reset(4096)
            self.nex_api.reload_all_programs()
            self.nex_api.enable_robot()
            self.nex_api.start_programs()
            while not rospy.is_shutdown():
                try:
                    if self.Stop_motion_flag == False:
                        self.pub_armstatus.publish(1)
                        if self.nex_api.task_state(0) == "Task exit": #if Task exit
                            self.Stop_motion_flag = True
                            self.pub_armstatus.publish(0)
                    else:
                        rospy.loginfo("Stop programs")
                        self.nex_api.stop_programs()
                        self.nex_api.disable_robot()
                        break
                except Exception, e:
                    self.Stop_motion_flag = False
                    rospy.logwarn("Could not running. %s", str(e))
                    raise e
        else:
            self.Stop_motion_flag = False
            rospy.loginfo("Please switch to external control mode")

    def arm_task_sub(self):
        if self.task_cmd == 1:
            self.task_cmd = 0
            self.arm_task_program()
        else:
            self.Stop_motion_flag = False
            
if __name__=="__main__":
    rospy.init_node("control_strategy")
    nex = nex_control()

    while not rospy.is_shutdown():
        nex.arm_task_sub()

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
