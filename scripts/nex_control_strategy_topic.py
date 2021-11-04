#!/usr/bin/env python
import rospy
from modbus.modbus_nex_api import ModbusNexApi 
from modbus.msg import peripheralCmd

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
class nex_control:
    def __init__(self):
        self.task_cmd = 0 # init number
        self.statusID = 0 # init number
        self.nex_api = ModbusNexApi()
        # rospy.loginfo("API setting")
        self.Stop_motion_flag = False
        self.Start_motion_flag = False
        self.pub_armstatus = rospy.Publisher("/reply_external_comm",peripheralCmd,queue_size=10)
        self.sub_taskcmd = rospy.Subscriber("/write_external_comm",peripheralCmd,self.callback)
        self.peripheralCmd = peripheralCmd()

    def init(self):
        self.task_cmd = 0 # init number
        self.statusID = 0 # init number
        self.Stop_motion_flag = False
        self.Start_motion_flag = False
        self.pub_armstatus = rospy.Publisher("/reply_external_comm",peripheralCmd,queue_size=10)
        self.sub_taskcmd = rospy.Subscriber("/write_external_comm",peripheralCmd,self.callback)
        self.peripheralCmd = peripheralCmd()

    def callback(self,data):
        self.task_cmd = data.actionTypeID
        self.statusID = data.statusID
        rospy.loginfo("I heard actionTypeID is %s", data.actionTypeID)
        rospy.loginfo("I heard statusID is %s", data.statusID)

    def stop_arm_reset(self):
        self.nex_api.stop_programs()
        self.nex_api.disable_robot()
        self.nex_api.send_reset(4096)

    def start_arm_reset(self):
        self.nex_api.send_reset_other_state(4096, 4) # reset and only reserve enable
        self.nex_api.reload_all_programs() 
        self.nex_api.stop_programs() # reset before starting cmd

        # TODO: this is not test 
        while not rospy.is_shutdown():
            if self.nex_api.is_task_init() == True:
                rospy.loginfo("reload_all_programs finished")
                break

    def publish_status_running(self):
        self.pub_armstatus.publish(99, 99)# hex 63, 63
    def publish_status_exit(self):
        self.pub_armstatus.publish(100, 100)# hex 64, 64
        
    def holding_registers_control(self, register, value):
        self.nex_api.modclient.setOutput(register,value,0)

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
                        self.nex_api.stop_programs()
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
                                    # TODO: add if arm error or stop 
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
                        self.nex_api.stop_programs()
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
                        self.nex_api.stop_programs()
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
                                    # TODO: add if arm error or stop 
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
                        self.nex_api.stop_programs()
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
                                    # TODO: add if arm error or stop 
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
        # TODO: switch function test 
        for case in switch(self.task_cmd):
            # start simple task_programs
            if case(1):
                self.task_cmd = 0
                self.arm_task_program()
                break
            # start rotation_task
            if case(2):
                self.task_cmd = 0
                self.rotation_task()
                break
            # start pick_and_place_task
            if case(3):
                self.task_cmd = 0
                self.pick_and_place_task()
                break
            # back_home
            if case(4):
                self.task_cmd = 0
                self.back_home_task()
                break
            # disable robot
            if case(5):
                self.task_cmd = 0
                self.nex_api.disable_robot()
                break
            # reset register address 4096
            if case(6):
                self.task_cmd = 0
                self.nex_api.send_reset(4096)
                self.nex_api.reset_error_robot()
                self.nex_api.send_reset(4096)
                break
            if case(): # default, could also just omit condition or 'if True'
		        # self.pub_armstatus.publish(100, 100)
                if self.task_cmd == 1003 and self.statusID == 99:
                    rospy.loginfo("Scripts break")
                    self.nex_api.stop_programs()
                    # self.stop_arm_reset()
                self.Stop_motion_flag = False

if __name__=="__main__":
    rospy.init_node("control_strategy")
    nex = nex_control()

    while not rospy.is_shutdown():
        nex.arm_task_sub()
