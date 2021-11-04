#!/usr/bin/env python
import rospy
from modbus.modbus_nex_api import ModbusNexApi 
from modbus.msg import peripheralCmd
class nex_control:
    def __init__(self):
        self.task_cmd = 0 # init number
        self.statusID = 0 # init number
        self.nex_api = ModbusNexApi()
        rospy.loginfo("API setting")
        self.Stop_motion_flag = False
        self.Start_motion_flag = False
        self.pub_armstatus = rospy.Publisher("/reply_external_comm",peripheralCmd,queue_size=10)
        self.sub_taskcmd = rospy.Subscriber("/write_external_comm",peripheralCmd,self.callback)
        self.peripheralCmd = peripheralCmd()

    def callback(self,data):
        self.task_cmd = data.actionTypeID
        self.statusID = data.statusID
        rospy.loginfo("I heard armTask_cmd is %s", data.actionTypeID)

    def stop_arm_reset(self):
        self.nex_api.stop_programs()
        self.nex_api.disable_robot()
        self.nex_api.send_reset(4096)

    def start_arm_reset(self):
        self.nex_api.send_reset_other_state(4096, 4) # reset and only reserve enable
        self.nex_api.reload_all_programs() 
        self.nex_api.stop_programs() # reset before starting cmd

    def publish_status_running(self):
        # self.peripheralCmd.statusID = 1
        # self.pub_armstatus.publish(0, self.peripheralCmd.statusID)
        self.pub_armstatus.publish(99, 99)
    def publish_status_exit(self):
        # self.peripheralCmd.statusID = 0
        self.pub_armstatus.publish(100, 100)
        self.pub_armstatus.publish(100, 100)
        self.pub_armstatus.publish(100, 100)
        self.pub_armstatus.publish(100, 100)
        self.pub_armstatus.publish(100, 100)

    def arm_task_program(self):
        ## Test all program run, can get task state
        if self.nex_api.operation_mode_state() == "EXT":
            i = 0
            self.start_arm_reset()
            start_status = self.nex_api.start_programs(0)
            while not rospy.is_shutdown():
                try:
                    # status 1003 AGV scripts close or shutdown
                    if self.task_cmd == 1003 and self.statusID == 99:
                        rospy.loginfo("Scripts break")
                        self.stop_arm_reset()
                        break
                    else:
                        if start_status == True:
                            if self.Stop_motion_flag == False:
                                # stop program command
                                if self.task_cmd == 5:
                                    self.task_cmd = 0
                                    self.stop_arm_reset()
                                    break
                                else:
                                    # task is running
                                    self.publish_status_running()

                                    i = i +1
                                    register = 1024
                                    self.nex_api.modclient.setOutput(register,i,0)
                                    print("i:",i)
                                    print("\n")
                                    rospy.sleep(2)
                                    # TODO: add If arm error or stop 
                                    if self.nex_api.task_state(0) == "Task exit": #if Task exit
                                        self.Stop_motion_flag = True
                                        # task is exit
                                        self.publish_status_exit()
                            else:
                                rospy.loginfo("Stop programs")
                                self.nex_api.stop_programs()
                                break
                        else:
                            break
                except Exception, e:
                    self.Stop_motion_flag = False
                    rospy.logwarn("Could not running. %s", str(e))
                    raise e
        else:
            self.Stop_motion_flag = False
            rospy.loginfo("Please switch to external control mode")

    def arm_task_sub(self):
        # start task_program
        if self.task_cmd == 1:
            self.task_cmd = 0
            self.arm_task_program()
        # enable robot
        elif self.task_cmd == 2:
            self.task_cmd = 0
            self.nex_api.enable_robot()
        # disable robot
        elif self.task_cmd == 3:
            self.task_cmd = 0
            self.nex_api.disable_robot()
        # reset register address 4096
        elif self.task_cmd == 4:
            self.task_cmd = 0
            self.nex_api.send_reset(4096)
        else:
            self.Stop_motion_flag = False
            
if __name__=="__main__":
    rospy.init_node("control_strategy")
    nex = nex_control()

    while not rospy.is_shutdown():
        nex.arm_task_sub()

# TODO: test select project function not test
# Test select project

    # nex_api.project_name("")
    # nex_api.open_project()

    # if nex_api.open_project_state() == "Opening":
    #     rospy.loginfo("opening ok")
    # else:
    #     rospy.loginfo("opening fail")
        
# TODO: Test all program run, can get task state
    # if nex_api.operation_mode_state() == "EXT":
    #     nex_api.send_reset(4096)
    #     nex_api.reload_all_programs()
    #     nex_api.enable_robot()

    #     # if nex_api.is_idle == True:
    #     nex_api.start_programs()
    #     while not rospy.is_shutdown():
    #         try:
    #             if Stop_motion_flag == False:
    #                 if nex_api.task_state(0) == "Task exit": #if Task exit
    #                     Stop_motion_flag = True
    #             else:
    #                 rospy.loginfo("Stop programs")
    #                 nex_api.stop_programs()
    #                 nex_api.disable_robot()
    #                 break
    #         except Exception, e:
    #             Stop_motion_flag = False
    #             rospy.logwarn("Could not running. %s", str(e))
    #             raise e
    #     # else:
    #     #     Stop_motion_flag = False
    #     #     rospy.loginfo("The robot is moving or not enable, please send the command again")
    # else:
    #     Stop_motion_flag = False
    #     rospy.loginfo("Please switch to external control mode")
# TODO: Test read input register test
    
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

    
#     a = np.array([0,-90,0,-90,0,0], dtype = np.float64)

#     # c = a.astype(np.float64)

#     print("a=", a)

#     i = 0
# # TODO: Test sent data holding register
#     while not rospy.is_shutdown():
#         i = i +1
#         register = 40001
#         nex_api.modclient.setOutput(register,i,0)
#         print("i:",i)
#         print("\n")
#         rospy.sleep(2)