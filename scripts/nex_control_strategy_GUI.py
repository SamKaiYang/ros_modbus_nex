#!/usr/bin/env python
# -*-coding:utf-8 -*-
import rospy
import threading
import time
import numpy as np
from modbus.modbus_nex_api import ModbusNexApi 
from modbus.msg import peripheralCmd
from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer, QThread, pyqtSignal
from main_ui import Ui_MainWindow
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
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
        rospy.loginfo("API setting")
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
class MyThread(QThread):
    callback = pyqtSignal(int, int)#自定義訊號, Qt的文件中有說明, 必需為類別變數
    def __init__(self, label, delay, parent=None):
        super(MyThread, self).__init__(parent)
        self.runFlag = True
        self.label=label
        self.delay=delay
        
    def __del__(self):
        self.runFlag = False
        self.wait()

    def run(self):
        index=0
        while self.runFlag:
            self.callback.emit(index, self.label)
            print(threading.currentThread().getName())
            index+=1
            self.msleep(self.delay)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.nex_api_ui = ModbusNexApi()
        self.nex_control = nex_control()
        self.ui.setupUi(self)
        self.ui.label_5.setText('Hello World!')
        self.ui.btn_reset.clicked.connect(self.reset_buttonClicked)
        self.ui.btn_enable.clicked.connect(self.enable_buttonClicked)
        self.ui.btn_disable.clicked.connect(self.disable_buttonClicked)
        self.ui.btn_reload.clicked.connect(self.reload_buttonClicked)
        self.ui.btn_start.clicked.connect(self.start_buttonClicked)
        self.ui.onBtn.clicked.connect(self.onBtn)

        # QTimer
        self.timer = QTimer()

        # QPushButton
        self.ui.btn_start_time.clicked.connect(self.timeGo)
        self.ui.btn_stop_time.clicked.connect(self.timeStop)

        # Other
        self.timer.timeout.connect(self.LCDEvent)
        self.s = 0

        # ComboBox
        choices = ['1', '2', '3', '4']
        self.ui.comboBox.addItems(choices)
        self.ui.comboBox.currentIndexChanged.connect(self.display)
        self.display()

    def display(self):
        self.ui.label_mission_case_show.setText('你目前選擇的是：%s' % self.ui.comboBox.currentText())

    def timeGo(self):
        self.timer.start(100)
        # self.ui.label_arm_picture.setPixmap(QtGui.QPixmap("../picture/teco_arm.png"))

    def timeStop(self):
        self.timer.stop()

    def LCDEvent(self):
        self.s += 1
        second = self.s/10
        m_second = self.s%10
        showtest = str(second) + '.' + str(m_second)
        self.ui.lcdNumber.display(showtest)

    def reset_buttonClicked(self):
        self.nex_api_ui.send_reset(4096)
        self.nex_api_ui.reset_error_robot()
        self.nex_api_ui.send_reset(4096)

    def enable_buttonClicked(self):
        self.nex_api_ui.send_reset_other_state(4096, 4) # reset and only reserve enable

    def disable_buttonClicked(self):
        self.nex_api_ui.disable_robot()

    def reload_buttonClicked(self):
        self.nex_control.start_arm_reset()

    def start_buttonClicked(self):
        start_status = self.nex_api_ui.start_programs(0)

    def onBtn(self, event):
        self.thread1=MyThread(1, 200)
        self.thread1.callback.connect(self.drawUi)
        self.thread1.start()

        self.thread2=MyThread(2, 1000)
        self.thread2.callback.connect(self.drawUi)
        self.thread2.start()

    #@QtCore.pyqtSlot(int, int)
    def drawUi(self, index, label):
        if label==1:
            self.ui.label_task_state.setText(self.nex_api_ui.task_state(0))
            self.ui.label_safety_state.setText(self.nex_api_ui.safety_state())
            self.ui.lbl1.setText(str(index))
        else :
            self.ui.lbl2.setText(str(index))
        print(threading.currentThread().getName())


		
if __name__=="__main__":
    rospy.init_node("control_strategy")
    nex = nex_control()
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()

    # while not rospy.is_shutdown():
    #     nex.arm_task_sub()

    sys.exit(app.exec_())
    # thread_ui.join()