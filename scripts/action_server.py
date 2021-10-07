#! /usr/bin/env python
import rospy
import actionlib
import modbus.msg

class ArmControlActionClass(object):
    # create messages that are used to publish feedback/result
    _feedback = modbus.msg.arm_task_commandFeedback()
    _result = modbus.msg.arm_task_commandResult()

    def __init__(self, name):
        # This will be the name of the action server which clients can use to connect to.
        self._action_name = name

        # Create a simple action server of the newly defined action type and
        # specify the execute callback where the goal will be processed.
        self._as = actionlib.SimpleActionServer(self._action_name, modbus.msg.arm_task_commandAction, execute_cb=self.execute_cb, auto_start = False)

        # Start the action server.
        self._as.start()
        rospy.loginfo("Action server started...")

    def execute_cb(self, goal):
        # Define frequency
        r = rospy.Rate(1)

        # Variable to decide the final state of the action server.
        success = True

    # start executing the action
        for counter_idx in range(0, goal.num_counts):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            # publish the feedback
            self._feedback.counts_elapsed = counter_idx
            self._as.publish_feedback(self._feedback)
            # Wait for counter_delay s before incrementing the counter.
            r.sleep()
            print("AA")

        if success:
            self._result.result_message = "Successfully completed counting."
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    # Initialize a ROS node for this action server.
    rospy.init_node('counter_with_delay')

    # Create an instance of the action server here.
    server = ArmControlActionClass(rospy.get_name())
    rospy.spin()
