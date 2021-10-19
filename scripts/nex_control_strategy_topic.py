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
            self.start_arm_reset()
            register = 1024
            self.nex_api.modclient.setOutput(register,1,0)
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

    def rotation_task(self):
        ## Test all program run, can get task state
        if self.nex_api.operation_mode_state() == "EXT":
            self.start_arm_reset()
            register = 1024
            self.nex_api.modclient.setOutput(register,2,0)
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

    def pick_and_place_task(self):
        ## Test all program run, can get task state
        if self.nex_api.operation_mode_state() == "EXT":
            self.start_arm_reset()
            register = 1024
            self.nex_api.modclient.setOutput(register,3,0)
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

    def back_home_task(self):
        ## Test all program run, can get task state
        if self.nex_api.operation_mode_state() == "EXT":
            self.start_arm_reset()
            register = 1024
            self.nex_api.modclient.setOutput(register,4,0)
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
        # start simple task_program
        if self.task_cmd == 1:
            self.task_cmd = 0
            self.arm_task_program()
        # start rotation_task
        elif self.task_cmd == 2:
            self.task_cmd = 0
            self.rotation_task()
        # start pick_and_place_task
        elif self.task_cmd == 3:
            self.task_cmd = 0
            self.pick_and_place_task()
        # back_home
        elif self.task_cmd == 4:
            self.task_cmd = 0
            self.back_home_task()
        # reset register address 4096
        elif self.task_cmd == 6:
            self.task_cmd = 0
            self.nex_api.send_reset(4096)
            self.nex_api.reset_error_robot()
            self.nex_api.send_reset(4096)
        else:
            self.Stop_motion_flag = False
            
if __name__=="__main__":
    rospy.init_node("control_strategy")
    nex = nex_control()

    while not rospy.is_shutdown():
        nex.arm_task_sub()