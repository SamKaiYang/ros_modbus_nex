#!/usr/bin/env python
import rospy
from modbus.modbus_nex_api import ModbusNexApi 
from std_msgs.msg import Int32MultiArray as HoldingRegister
import actionlib
import modbus.msg

class ArmControlActionClass(object):
    # create messages that are used to publish feedback/result
    _feedback = modbus.msg.arm_task_commandFeedback()
    _result = modbus.msg.arm_task_commandResult()

    def __init__(self, name):
        # This will be the name of the action server which clients can use to connect to.
        self._action_name = name
        self.Stop_motion_flag = False
        self.Start_motion_flag = False   
        # Create a simple action server of the newly defined action type and
        # specify the execute callback where the goal will be processed.
        self._as = actionlib.SimpleActionServer(self._action_name, modbus.msg.arm_task_commandAction, execute_cb=self.execute_cb, auto_start = False)

        # Start the action server.
        self._as.start()
        rospy.loginfo("Action server started...")

    def execute_cb(self, goal):
        self.start_arm_task(goal.cmd)

    def start_arm_task(self, cmd):
        if cmd == 1:
            ## Test all program run, can get task state
            if nex_api.operation_mode_state() == "EXT":
                
                nex_api.start_programs()
                while not rospy.is_shutdown():
                    # check that preempt has not been requested by the client
                    if self._as.is_preempt_requested():
                        rospy.loginfo('%s: Preempted' % self._action_name)
                        self._as.set_preempted()
                        # success = False
                        break
                    try:
                        if self.Stop_motion_flag == False:
                            if nex_api.task_state(0) == "Task exit": #if Task exit
                                self.Stop_motion_flag = True
                        else:
                            rospy.loginfo("Stop programs")
                            nex_api.stop_programs()
                            # send result to client
                            self._result.result_message = "Successfully completed counting."
                            rospy.loginfo('%s: Succeeded' % self._action_name)
                            self._as.set_succeeded(self._result)
                            # nex_api.disable_robot()
                            self.Stop_motion_flag = False
                            break
                    except Exception, e:
                        self.Stop_motion_flag = False
                        rospy.logwarn("Could not running. %s", str(e))
                        raise e
            else:
                self.Stop_motion_flag = False
                rospy.loginfo("Please switch to external control mode")
        if cmd == 2:
            nex_api.disable_robot()
            self._result.result_message = "disable_arm_done"
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        if cmd == 3:
            nex_api.enable_robot()
            self._result.result_message = "enable_arm_done"
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
if __name__=="__main__":
    rospy.init_node("control_strategy")
    # Create an instance of the action server here.
    nex_api = ModbusNexApi()
    rospy.loginfo("API setting")

    nex_api.send_reset(4096)
    nex_api.reload_all_programs()
    nex_api.enable_robot()

    server = ArmControlActionClass(rospy.get_name())

    rospy.spin()