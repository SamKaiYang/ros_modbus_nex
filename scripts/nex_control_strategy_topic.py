#!/usr/bin/env python
import rospy
from modbus.modbus_nex_api import ModbusNexApi 
from modbus.msg import peripheralCmd
class nex_control:
    def __init__(self):
        self.task_cmd = 0 # init number
        self.nex_api = ModbusNexApi()
        rospy.loginfo("API setting")
        self.Stop_motion_flag = False
        self.Start_motion_flag = False
        self.pub_armstatus = rospy.Publisher("/reply_external_comm",peripheralCmd,queue_size=10)
        self.sub_taskcmd = rospy.Subscriber("/write_external_comm",peripheralCmd,self.callback)
        self.peripheralCmd = peripheralCmd()
    def callback(self,data):
        self.task_cmd = data.actionTypeID
        rospy.loginfo("I heard armTask_cmd is %s", data.actionTypeID)

    # TODO: start enable robot shutdown disable robot, not loop enable & disable
    def arm_task_program(self):
        ## Test all program run, can get task state
        if self.nex_api.operation_mode_state() == "EXT":
            
            # TODO: test 1013
            if self.nex_api.is_enabled() == False: 
                self.nex_api.enable_robot()

            # self.nex_api.send_reset(4096)
            self.nex_api.reload_all_programs() # TODO: test 1013
            # self.nex_api.enable_robot()
            self.nex_api.start_programs()
            while not rospy.is_shutdown():
                try:
                    if self.Stop_motion_flag == False:
                        # task is running
                        self.peripheralCmd.statusID = 1
                        self.pub_armstatus.publish(0, self.peripheralCmd.statusID)
                        if self.nex_api.task_state(0) == "Task exit": #if Task exit
                            self.Stop_motion_flag = True
                            # task is exit
                            self.peripheralCmd.statusID = 0
                            self.pub_armstatus.publish(0, self.peripheralCmd.statusID)
                    else:
                        rospy.loginfo("Stop programs")
                        self.nex_api.stop_programs() # TODO: test 1013
                        # self.nex_api.disable_robot()
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
        # # reload_all_programs
        # elif self.task_cmd == 2:
        #     self.task_cmd = 0
        #     self.nex_api.send_reset(4096)
        #     self.nex_api.reload_all_programs()
        # enable robot
        elif self.task_cmd == 3:
            self.task_cmd = 0
            self.nex_api.enable_robot()
        # disable robot
        elif self.task_cmd == 4:
            self.task_cmd = 0
            self.nex_api.disable_robot()
        else:
            self.Stop_motion_flag = False
            
if __name__=="__main__":
    rospy.init_node("control_strategy")
    nex = nex_control()

    while not rospy.is_shutdown():
        nex.arm_task_sub()