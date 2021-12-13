#!/usr/bin/env python
# -*-coding:utf-8 -*-
import rospy
from modbus.modbus_nex_api import ModbusNexApi 
from modbus.msg import peripheralCmd, ipconfig, closenode

class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration
    
    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False

class Nex_control(ModbusNexApi):
    def init(self):
        self.task_cmd = 0 # init number
        self.statusID = 0 # init number
        self.close_node = 0
        self.Stop_motion_flag = False
        self.Start_motion_flag = False
        self.pub_armstatus = rospy.Publisher("/reply_external_comm",peripheralCmd,queue_size=10)
        self.sub_taskcmd = rospy.Subscriber("/write_external_comm",peripheralCmd,self.callback)
        self.sub_ipset = rospy.Subscriber("/ip_comm",ipconfig,self.ip_callback)
        self.sub_close_node = rospy.Subscriber("/close_node",closenode,self.close_node_callback)
        self.peripheralCmd = peripheralCmd()
        self.ipconfig = ipconfig()

    def callback(self,data):
        self.task_cmd = data.actionTypeID
        self.statusID = data.statusID
        rospy.loginfo("I heard actionTypeID is %s", data.actionTypeID)
        rospy.loginfo("I heard statusID is %s", data.statusID)

    def ip_callback(self,data):
        self.ip_set(data.ip)
        rospy.loginfo("strategy ip set finished")

    def close_node_callback(self, data):
        self.close_node = data.signal
        if self.close_node == 1:
            reason = "You sure to quit"
            rospy.signal_shutdown(reason)
        rospy.loginfo("strategy close")

    def stop_arm_reset(self):
        self.stop_programs()
        self.disable_robot()
        self.send_reset(4096)

    def start_arm_reset(self):
        self.send_reset_other_state(4096, 4) # reset and only reserve enable
        self.reload_all_programs() 
        self.stop_programs() # reset before starting cmd

        while not rospy.is_shutdown():
            if self.is_task_init() == True:
                rospy.loginfo("reload_all_programs finished")
                break

    def publish_status_running(self):
        self.pub_armstatus.publish(99, 99)# hex 63, 63
    def publish_status_exit(self):
        self.pub_armstatus.publish(100, 100)# hex 64, 64
        
    def holding_registers_control(self, register, value):
        self.modclient.setOutput(register,value,0)

    def arm_task_program(self):
        ## Test all program run, can get task state
        if self.operation_mode_state() == "EXT":
            self.start_arm_reset()
            register = 1024
            self.modclient.setOutput(register,1,0)
            start_status = self.start_programs(0)
            while not rospy.is_shutdown():
                try:
                    # status 1003 AGV scripts close or shutdown
                    if self.task_cmd == 1003 and self.statusID == 99:
                        rospy.loginfo("Scripts break")
                        self.stop_programs()
                        # self.stop_arm_reset()
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
                                    if self.task_state(0) == "Task exit": #if Task exit
                                        self.Stop_motion_flag = True
                                        # task is exit
                                        self.publish_status_exit()
                            else:
                                rospy.loginfo("Stop programs")
                                self.stop_programs()
                                break
                        else:
                            break
                except Exception as e:
                    self.Stop_motion_flag = False
                    rospy.logwarn("Could not running. %s", str(e))
                    raise e
        else:
            self.Stop_motion_flag = False
            rospy.loginfo("Please switch to external control mode")

    def smile_task(self):
        ## Test all program run, can get task state
        if self.operation_mode_state() == "EXT":
            self.start_arm_reset()
            register = 1024
            self.modclient.setOutput(register,2,0)
            start_status = self.start_programs(0)
            while not rospy.is_shutdown():
                try:
                    # status 1003 AGV scripts close or shutdown
                    if self.task_cmd == 1003 and self.statusID == 99:
                        rospy.loginfo("Scripts break")
                        self.stop_programs()
                        # self.stop_arm_reset()
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
                                    if self.task_state(0) == "Task exit": #if Task exit
                                        self.Stop_motion_flag = True
                                        # task is exit
                                        self.publish_status_exit()
                            else:
                                rospy.loginfo("Stop programs")
                                self.stop_programs()
                                break
                        else:
                            break
                except Exception as e:
                    self.Stop_motion_flag = False
                    rospy.logwarn("Could not running. %s", str(e))
                    raise e
        else:
            self.Stop_motion_flag = False
            rospy.loginfo("Please switch to external control mode")

    def bow_task(self):
        ## Test all program run, can get task state
        if self.operation_mode_state() == "EXT":
            self.start_arm_reset()
            register = 1024
            self.modclient.setOutput(register,7,0)
            start_status = self.start_programs(0)
            while not rospy.is_shutdown():
                try:
                    # status 1003 AGV scripts close or shutdown
                    if self.task_cmd == 1003 and self.statusID == 99:
                        rospy.loginfo("Scripts break")
                        self.stop_programs()
                        # self.stop_arm_reset()
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
                                    if self.task_state(0) == "Task exit": #if Task exit
                                        self.Stop_motion_flag = True
                                        # task is exit
                                        self.publish_status_exit()
                            else:
                                rospy.loginfo("Stop programs")
                                self.stop_programs()
                                break
                        else:
                            break
                except Exception as e:
                    self.Stop_motion_flag = False
                    rospy.logwarn("Could not running. %s", str(e))
                    raise e
        else:
            self.Stop_motion_flag = False
            rospy.loginfo("Please switch to external control mode")
    def show_task(self):
        ## Test all program run, can get task state
        if self.operation_mode_state() == "EXT":
            self.start_arm_reset()
            register = 1024
            self.modclient.setOutput(register,3,0)
            start_status = self.start_programs(0)
            while not rospy.is_shutdown():
                try:
                    # status 1003 AGV scripts close or shutdown
                    if self.task_cmd == 1003 and self.statusID == 99:
                        rospy.loginfo("Scripts break")
                        self.stop_programs()
                        # self.stop_arm_reset()
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
                                    if self.task_state(0) == "Task exit": #if Task exit
                                        self.Stop_motion_flag = True
                                        # task is exit
                                        self.publish_status_exit()
                            else:
                                rospy.loginfo("Stop programs")
                                self.stop_programs()
                                break
                        else:
                            break
                except Exception as e:
                    self.Stop_motion_flag = False
                    rospy.logwarn("Could not running. %s", str(e))
                    raise e
        else:
            self.Stop_motion_flag = False
            rospy.loginfo("Please switch to external control mode")

    def back_home_task(self):
        ## Test all program run, can get task state
        if self.operation_mode_state() == "EXT":
            self.start_arm_reset()
            register = 1024
            self.modclient.setOutput(register,4,0)
            start_status = self.start_programs(0)
            while not rospy.is_shutdown():
                try:
                    # status 1003 AGV scripts close or shutdown
                    if self.task_cmd == 1003 and self.statusID == 99:
                        rospy.loginfo("Scripts break")
                        self.stop_programs()
                        # self.stop_arm_reset()
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
                                    if self.task_state(0) == "Task exit": #if Task exit
                                        self.Stop_motion_flag = True
                                        # task is exit
                                        self.publish_status_exit()
                            else:
                                rospy.loginfo("Stop programs")
                                self.stop_programs()
                                break
                        else:
                            break
                except Exception as e:
                    self.Stop_motion_flag = False
                    rospy.logwarn("Could not running. %s", str(e))
                    raise e
        else:
            self.Stop_motion_flag = False
            rospy.loginfo("Please switch to external control mode")

    def arm_task_sub(self):
        for case in switch(self.task_cmd):
            # start simple task_programs
            if case(1):
                self.task_cmd = 0
                self.arm_task_program()
                break
            # start smile_task # show smile
            if case(2):
                self.task_cmd = 0
                self.smile_task()
                break
            # start show dance
            if case(3):
                self.task_cmd = 0
                self.show_task()
                break
            # back_home
            if case(4):
                self.task_cmd = 0
                self.back_home_task()
                break
            # disable robot
            if case(5):
                self.task_cmd = 0
                self.disable_robot()
                break
            # reset register address 4096
            if case(6):
                self.task_cmd = 0
                self.send_reset(4096)
                self.reset_error_robot()
                self.send_reset(4096)
                break
            # start bow_task # show bow
            if case(7):
                self.task_cmd = 0
                self.bow_task()
                break
            if case(): # default, could also just omit condition or 'if True'
                # self.pub_armstatus.publish(100, 100)
                if self.task_cmd == 1003 and self.statusID == 99:
                    self.task_cmd = 0
                    rospy.loginfo("Scripts break")
                    self.stop_programs()
                    # self.stop_arm_reset()
                self.Stop_motion_flag = False

if __name__=="__main__":
    rospy.init_node("control_strategy")
    nex = Nex_control()
    nex.init()
    while not rospy.is_shutdown():
        nex.arm_task_sub()
        pass
