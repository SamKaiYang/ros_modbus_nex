#!/usr/bin/env python
# -*-coding:utf-8 -*-
import rospy
import threading
import time
import numpy as np
from modbus.modbus_nex_api import ModbusNexApi 
from modbus.msg import peripheralCmd
from PyQt5 import QtWidgets, QtGui
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

        # TODO: this is not test 
        while not rospy.is_shutdown():
            if self.nex_api.is_task_init() == True:
                rospy.loginfo("reload_all_programs finished")
                break #
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
            # print(threading.currentThread().getName())
            # TODO：移除index
            index+=1
            self.msleep(self.delay)

class StrategyThread(QThread):
    callback = pyqtSignal(int, int)#自定義訊號, Qt的文件中有說明, 必需為類別變數
    def __init__(self, label, delay, parent=None):
        super(StrategyThread, self).__init__(parent)
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
            # print(threading.currentThread().getName())
            # TODO：移除index
            index+=1
            self.msleep(self.delay)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.vel = 50
        self.acc = 50
        
        self.ui = Ui_MainWindow()
        self.nex_api_ui = ModbusNexApi()
        self.nex_control = nex_control()
        self.mission_number = 0
        self.ui.setupUi(self)
        self._creat_menubar()
        self.setWindowIcon(QtGui.QIcon('src/modbus/modbus/picture/teco_icon.png'))
        self.ui.lineEdit_vel.setText(str(self.vel))
        self.ui.lineEdit_acc.setText(str(self.acc))
        self.ui.btn_reset.clicked.connect(self.reset_buttonClicked)
        self.ui.btn_enable.clicked.connect(self.enable_buttonClicked)
        self.ui.btn_disable.clicked.connect(self.disable_buttonClicked)
        self.ui.btn_reload.clicked.connect(self.reload_buttonClicked)
        self.ui.onBtn.clicked.connect(self.onBtn)
        self.ui.btn_start_program.clicked.connect(self.start_buttonClicked)
        self.ui.btn_vel_set.clicked.connect(self.vel_setClicked)
        self.ui.btn_acc_set.clicked.connect(self.acc_setClicked)
        self.ui.btn_ip_set.clicked.connect(self.ip_setClicked)
        
        # TODO: edit thread for AGV & arm strategy 
        # self.thread1=StrategyThread(1, 100)
        # self.thread1.callback.connect(self.drawUi)
        # self.thread1.start()

        # Vel. HorizontalSlider
        self.ui.horizontalSlider_vel.valueChanged.connect(self.VelSliderValue)
        # Acc. HorizontalSlider
        self.ui.horizontalSlider_acc.valueChanged.connect(self.AccSliderValue)
        # QTimer
        self.timer = QTimer()

        # QPushButton
        self.ui.btn_start_time.clicked.connect(self.timeGo)
        self.ui.btn_stop_time.clicked.connect(self.timeStop)
        self.ui.btn_reset_time.clicked.connect(self.timeReset)

        # Other
        self.timer.timeout.connect(self.LCDEvent)
        self.s = 0

        # ComboBox
        choices = ['None','All', 'Init', '3', 'Home','5','6']
        self.ui.comboBox.addItems(choices)
        self.ui.comboBox.currentIndexChanged.connect(self.display)
        self.display()
    
        self.initUi()

        # Gif 
        self.movie = QtGui.QMovie("src/modbus/modbus/picture/earth.gif")
        self.ui.label_arm_gif.setMovie(self.movie)
        self.movie.start()

    def initUi(self):
        self.status = self.statusBar()
        self.status.showMessage('Update State', 0) #状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
        self.status.setStyleSheet("font-size: 18px;background-color: #F5E8FF")
        self.safetyNum = QtWidgets.QLabel("Safety:")
        self.taskNum = QtWidgets.QLabel("Task:")
        self.reloadNum = QtWidgets.QLabel("Reload:")

        self.safetyNum.setFixedWidth(200)
        # self.safetyNum.setStyleSheet("background-color:red;font-size: 18px;border-radius: 25px;border: 1px solid black;")
        self.safetyNum.setStyleSheet("font-size: 18px;border-radius: 25px;border: 1px solid black;")
        self.taskNum.setFixedWidth(200)
        self.taskNum.setStyleSheet("font-size: 18px;border-radius: 25px;border: 1px solid black;")
        self.reloadNum.setFixedWidth(200)
        self.reloadNum.setStyleSheet("font-size: 18px;border-radius: 25px;border: 1px solid black;")

        self.status.addPermanentWidget(self.safetyNum, stretch=0)
        self.status.addPermanentWidget(self.taskNum, stretch=0)
        self.status.addPermanentWidget(self.reloadNum, stretch=0)
        
    def _creat_menubar(self):
        self.menu=self.menuBar()
        file=self.menu.addMenu('File')
        # file.addAction('New')
        # file.addAction('Open')
        # file.addAction('Close Project')

        tool=self.menu.addMenu('Tool')
        # tool.addAction('Python')
        # tool.addAction('C++')
        # tool.addAction('C')

    def display(self):
        self.ui.label_mission_case_show.setText('Choose：%s' % self.ui.comboBox.currentText())
        if self.ui.comboBox.currentText() == "None":
            task_value = 0
        elif self.ui.comboBox.currentText() == "All":
            task_value = 1
        elif self.ui.comboBox.currentText() == "Init":
            task_value = 2
        elif self.ui.comboBox.currentText() == "3":
            task_value = 3
        elif self.ui.comboBox.currentText() == "Home":
            task_value = 4
        elif self.ui.comboBox.currentText() == "5":
            task_value = 5
        elif self.ui.comboBox.currentText() == "6":
            task_value = 6
        self.mission_number = task_value

    def timeGo(self):
        self.timer.start(100)
        
    def timeStop(self):
        self.timer.stop()

    def timeReset(self):
        # self.timer.reset()
        self.s = 0
        second = self.s/10
        m_second = self.s%10
        showtest = str(second) + '.' + str(m_second)
        self.ui.lcdNumber.display(showtest)

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
        # self.nex_control.start_arm_reset()
        self.nex_api_ui.send_reset_other_state(4096, 4) # reset and only reserve enable
        self.nex_api_ui.reload_all_programs() 
        self.nex_api_ui.stop_programs() # reset before starting cmd
        while not rospy.is_shutdown():
            if self.nex_api_ui.is_task_init() == True:
                rospy.loginfo("reload_all_programs finished")
                break #

    def start_buttonClicked(self):
        register = 1024
        value = self.mission_number
        self.nex_api_ui.modclient.setOutput(register,value,0)
        start_status = self.nex_api_ui.start_programs(0)

    def onBtn(self, event):
        self.thread1=MyThread(1, 100)
        self.thread1.callback.connect(self.drawUi)
        self.thread1.start()

        self.thread2=MyThread(2, 100)
        self.thread2.callback.connect(self.drawUi)
        self.thread2.start()

    def ip_setClicked(self):
        text = self.ui.lineEdit_ip.text()
        self.ui.label_ip.setText("Set IP:"+text)
        self.ui.lineEdit_ip.clear()
        self.nex_api_ui.ip_set(text)
        rospy.loginfo("Setup complete")

    #@QtCore.pyqtSlot(int, int)
    def drawUi(self, index, label):
        if label==1:
            # self.ui.label_task_state.setText("task state:"+self.nex_api_ui.task_state(0))
            # self.ui.label_reload_state.setText("reload state:"+self.nex_api_ui.is_task_init())
            self.safetyNum.setText("Safety:"+self.safety_state())
            self.taskNum.setText("Task:"+self.task_state(0))
            self.reloadNum.setText("Reload:"+str(self.is_task_init()))
            # (0)Disable, (1)Ready, (2)Error, (3)Enable, (4)Running
            if self.nex_api_ui.safety_state() == "Error":
                self.safetyNum.setStyleSheet("background-color:red;font-size: 18px;border-radius: 25px;border: 1px solid black;")
        else :
            # ACS_actual = self.nex_api_ui.read_ACS_actual_position()
            # ACS_command = self.nex_api_ui.read_ACS_command_position()
            # PCS_actual = self.nex_api_ui.read_PCS_actual_position()
            # PCS_command = self.nex_api_ui.read_PCS_command_position()
            # self.ui.label_acs_command_show.setText("A1:"+ ACS_command.axis1 +"A2:" + ACS_command.axis2 + "A3:" + ACS_command.axis3 + "A4:"+ ACS_command.axis4 +  "A5:"+ ACS_command.axis5 + "A6:" + ACS_command.axis6 )
            # self.ui.label_acs_actual_show.setText("A1:"+ ACS_actual.axis1 +"A2:" + ACS_actual.axis2 + "A3:" + ACS_actual.axis3 + "A4:"+ ACS_actual.axis4 +  "A5:"+ ACS_actual.axis5 + "A6:" + ACS_actual.axis6 )
            # self.ui.label_pcs_command_show.setText("X:"+ PCS_command.X +"Y:" + PCS_command.Y + "Z:" + PCS_command.Z + "A:"+ PCS_command.A +  "B:"+ PCS_command.B + "C:" + PCS_command.C )
            # self.ui.label_pcs_actual_show.setText("X:"+ PCS_actual.X +"Y:" + PCS_actual.Y + "Z:" + PCS_actual.Z + "A:"+ PCS_actual.A +  "B:"+ PCS_actual.B + "C:" + PCS_actual.C )
            pass

    def vel_setClicked(self):
        register = 1025
        self.vel = int(self.ui.lineEdit_vel.text())
        self.nex_api_ui.modclient.setOutput(register,self.vel,0)

    def acc_setClicked(self):
        register = 1026
        self.acc = int(self.ui.lineEdit_acc.text())
        self.nex_api_ui.modclient.setOutput(register,self.acc,0)

    def VelSliderValue(self):
        self.ui.lineEdit_vel.setText(str(self.ui.horizontalSlider_vel.value()))
        
    def AccSliderValue(self):
        self.ui.lineEdit_acc.setText(str(self.ui.horizontalSlider_acc.value()))

if __name__=="__main__":
    rospy.init_node("control_strategy")
    nex = nex_control()
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
    # thread_ui.join()