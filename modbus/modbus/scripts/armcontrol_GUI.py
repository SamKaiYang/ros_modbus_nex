#!/usr/bin/env python
# -*-coding:utf-8 -*-
import rospy
# import threading
# import time
import numpy as np
from modbus.modbus_nex_api import ModbusNexApi 
from modbus.msg import peripheralCmd, ipconfig, closenode, joint_state

from PySide2 import QtWidgets, QtGui
from PySide2.QtCore import *
from PySide2.QtWidgets import *
from PySide2.QtGui import *
from Ui_main import Ui_MainWindow

# from sensor_msgs.msg import JointState
from std_msgs.msg import Header
# from armcontrol_strategy import Nex_control, switch
import sys
reload(sys)
#import importlib,sys
#importlib.reload(sys)
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

class MyThread(QThread):
    callback = Signal(int, int)#自定義訊號, Qt的文件中有說明, 必需為類別變數
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
            index+=1
            self.msleep(self.delay)
class PandasModel_pos(QAbstractTableModel):
    header_labels = ['X', 'Y', 'Z', 'A', 'B', 'C']
    vertical_labels = ['Command', 'Actual','Unit']
    def __init__(self, data, parent=None):
        super(PandasModel_pos,self).__init__(parent)
        self._data = data

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return self.header_labels[section]
        if role == Qt.DisplayRole and orientation == Qt.Vertical:
            return self.vertical_labels[section]
        return QAbstractTableModel.headerData(self, section, orientation, role)

    def rowCount(self, index):
        return self._data.shape[0]

    def columnCount(self, index):
        return self._data.shape[1]

    def data(self, index, role=Qt.DisplayRole):
        if index.isValid():
            if role == Qt.DisplayRole or role == Qt.EditRole:
                value = self._data[index.row(), index.column()]
                return str(value)

    def setData(self, index, value, role):
        if role == Qt.EditRole:
            try:
                value = int(value)
            except ValueError:
                return False
            self._data[index.row(), index.column()] = value
            return True
        return False

    def flags(self, index):
        return Qt.ItemIsSelectable | Qt.ItemIsEnabled | Qt.ItemIsEditable

class PandasModel_joint(QAbstractTableModel):
    header_labels = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
    vertical_labels = ['Command', 'Actual','Unit']
    def __init__(self, data, parent=None):
        super(PandasModel_joint,self).__init__(parent)
        self._data = data

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return self.header_labels[section]
        if role == Qt.DisplayRole and orientation == Qt.Vertical:
            return self.vertical_labels[section]
        return QAbstractTableModel.headerData(self, section, orientation, role)

    def rowCount(self, index):
        return self._data.shape[0]

    def columnCount(self, index):
        return self._data.shape[1]

    def data(self, index, role=Qt.DisplayRole):
        if index.isValid():
            if role == Qt.DisplayRole or role == Qt.EditRole:
                value = self._data[index.row(), index.column()]
                return str(value)

    def setData(self, index, value, role):
        if role == Qt.EditRole:
            try:
                value = int(value)
            except ValueError:
                return False
            self._data[index.row(), index.column()] = value
            return True
        return False

    def flags(self, index):
        return Qt.ItemIsSelectable | Qt.ItemIsEnabled | Qt.ItemIsEditable
class PandasModel_pub(QAbstractTableModel):
    header_labels = ['task_cmd', 'statusID']
    # vertical_labels = ['data', 'data','data']
    def __init__(self, data, parent=None):
        super(PandasModel_pub,self).__init__(parent)
        self._data = data

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return self.header_labels[section]
        # if role == Qt.DisplayRole and orientation == Qt.Vertical:
        #     return self.vertical_labels[section]
        return QAbstractTableModel.headerData(self, section, orientation, role)

    def rowCount(self, index):
        return self._data.shape[0]

    def columnCount(self, index):
        return self._data.shape[1]

    def data(self, index, role=Qt.DisplayRole):
        if index.isValid():
            if role == Qt.DisplayRole or role == Qt.EditRole:
                value = self._data[index.row(), index.column()]
                return str(value)

    def setData(self, index, value, role):
        if role == Qt.EditRole:
            try:
                value = int(value)
            except ValueError:
                return False
            self._data[index.row(), index.column()] = value
            return True
        return False

    def flags(self, index):
        return Qt.ItemIsSelectable | Qt.ItemIsEnabled | Qt.ItemIsEditable
class PandasModel_echo(QAbstractTableModel):
    header_labels = ['task_cmd', 'statusID']
    # vertical_labels = ['data', 'data','data']
    def __init__(self, data, parent=None):
        super(PandasModel_echo,self).__init__(parent)
        self._data = data

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return self.header_labels[section]
        # if role == Qt.DisplayRole and orientation == Qt.Vertical:
        #     return self.vertical_labels[section]
        return QAbstractTableModel.headerData(self, section, orientation, role)

    def rowCount(self, index):
        return self._data.shape[0]

    def columnCount(self, index):
        return self._data.shape[1]

    def data(self, index, role=Qt.DisplayRole):
        if index.isValid():
            if role == Qt.DisplayRole or role == Qt.EditRole:
                value = self._data[index.row(), index.column()]
                return str(value)

    def setData(self, index, value, role):
        if role == Qt.EditRole:
            try:
                value = int(value)
            except ValueError:
                return False
            self._data[index.row(), index.column()] = value
            return True
        return False

    def flags(self, index):
        return Qt.ItemIsSelectable | Qt.ItemIsEnabled | Qt.ItemIsEditable
class MainWindow(QtWidgets.QMainWindow, ModbusNexApi):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.vel = 50
        self.acc = 50
        self.checkflag = True
        self.ui = Ui_MainWindow()
        self.mission_number = 0
        self.task_cmd = 0 # init number
        self.statusID = 0 # init number
        self.task_cmd_reply = 0
        self.statusID_reply = 0
        self.jogging_postion_rel = 0
        self.jogging_posture_rel = 0
        self.pub_ipset = rospy.Publisher("/ip_comm",ipconfig,queue_size=10)
        self.pub_closenode = rospy.Publisher("/close_node",closenode,queue_size=10)
        self.pub_joint_state = rospy.Publisher("modbus_joint_states",joint_state,queue_size=10)
        self.joint_state = joint_state()
        
        self.startthreadflag = False
        self.ui.setupUi(self)
        self.setStyleSheet()
        self._creat_menubar()
        self.setWindowIcon(QtGui.QIcon('../picture/teco_icon.png'))
        self.ui.lineEdit_vel.setText(str(self.vel))
        # self.ui.lineEdit_acc.setText(str(self.acc))
        self.ui.btn_reset.clicked.connect(self.reset_buttonClicked)
        self.ui.btn_enable.clicked.connect(self.enable_buttonClicked)
        self.ui.btn_disable.clicked.connect(self.disable_buttonClicked)
        # self.ui.btn_reload.clicked.connect(self.reload_buttonClicked)
        self.ui.onBtn.clicked.connect(self.onBtn)
        self.ui.offBtn.clicked.connect(self.offBtn)
        self.ui.btn_start_program.clicked.connect(self.start_buttonClicked)
        self.ui.btn_stop_program.clicked.connect(self.stop_buttonClicked)
        # set arm speed 
        self.ui.btn_vel_set.clicked.connect(self.vel_setClicked)
        # self.ui.btn_acc_set.clicked.connect(self.acc_setClicked)
        self.ui.btn_vel_slow_set.clicked.connect(self.vel_slow_setClicked)
        self.ui.btn_vel_quick_set.clicked.connect(self.vel_quick_setClicked)
        

        self.ui.btn_ip_set.clicked.connect(self.ip_setClicked)
        # TPUI project 
        self.ui.btn_project_name_select.clicked.connect(self.project_name_setClicked)
        self.ui.btn_project_name_read.clicked.connect(self.project_name_readClicked)
        # arm jogging button
        self.ui.btn_x_positive.clicked.connect(lambda:self.pose_setClicked("+x"))
        self.ui.btn_x_negative.clicked.connect(lambda:self.pose_setClicked("-x"))
        self.ui.btn_y_positive.clicked.connect(lambda:self.pose_setClicked("+y"))
        self.ui.btn_y_negative.clicked.connect(lambda:self.pose_setClicked("-y"))
        self.ui.btn_z_positive.clicked.connect(lambda:self.pose_setClicked("+z"))
        self.ui.btn_z_negative.clicked.connect(lambda:self.pose_setClicked("-z"))
        self.ui.btn_a_positive.clicked.connect(lambda:self.pose_setClicked("+a"))
        self.ui.btn_a_negative.clicked.connect(lambda:self.pose_setClicked("-a"))
        self.ui.btn_b_positive.clicked.connect(lambda:self.pose_setClicked("+b"))
        self.ui.btn_b_negative.clicked.connect(lambda:self.pose_setClicked("-b"))
        self.ui.btn_c_positive.clicked.connect(lambda:self.pose_setClicked("+c"))
        self.ui.btn_c_negative.clicked.connect(lambda:self.pose_setClicked("-c"))
        # show program
        self.ui.btn_show_program.clicked.connect(self.show_program_readClicked)
        self.ui.btn_smile_program.clicked.connect(self.smile_program_readClicked)
        self.ui.btn_dance_program.clicked.connect(self.dance_program_readClicked)
        self.ui.btn_bow_program.clicked.connect(self.bow_program_readClicked)
        self.ui.btn_stopshow_program.clicked.connect(self.stopshow_program_readClicked)
        self.ui.btn_init_program.clicked.connect(self.init_program_readClicked)
        self.ui.btn_home_program.clicked.connect(self.home_program_readClicked)
        # test button 
        self.ui.btn_test.clicked.connect(self.testClicked)
        # self.ui.btn_test2.clicked.connect(self.test2Clicked)

        # modbus connect server ip init
        self.ip_init()
        # arm position class define
        self.point_init() 
        # Vel. HorizontalSlider
        self.ui.horizontalSlider_vel.valueChanged.connect(self.VelSliderValue)
        # Acc. HorizontalSlider
        # self.ui.horizontalSlider_acc.valueChanged.connect(self.AccSliderValue)
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
        # choices = ['None','Show 1', 'Show 2', 'Init', 'Home','Stop','Show All Select','jogging']
        choices = ['None','Show 1', 'Show 2', 'Smile', 'Bow', 'Home','Stop','Show All Select']
        self.ui.comboBox.addItems(choices)
        self.ui.comboBox.currentIndexChanged.connect(self.display)
        self.display()
        
        # ComboBox jog
        choices = ['None','100 mm / 10o','10 mm / 3o', '1 mm / 1o', '0.1 mm / 0.005o']
        self.ui.comboBox_jogging_mode.addItems(choices)
        self.ui.comboBox_jogging_mode.currentIndexChanged.connect(self.JogMode_display)
        self.JogMode_display()
        
        self.initUi()

        # Gif 
        self.movie = QtGui.QMovie("src/modbus/modbus/picture/earth.gif")
        self.ui.label_arm_gif.setMovie(self.movie)
        self.movie.start()
        # table_view init
        data_pos = np.array([
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            ["mm","mm","mm","Deg","Deg","Deg"]
        ])
        data_joint = np.array([
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            ["Deg","Deg","Deg","Deg","Deg","Deg"]
        ])
        self.data_pub = np.array([
            [0, 0]
        ])
        self.data_echo = np.array([
            [0, 0]
        ])
        self.model = PandasModel_pos(data_pos)
        self.ui.tableView_pos.setModel(self.model)
        self.model = PandasModel_joint(data_joint)
        self.ui.tableView_joint.setModel(self.model)
        self.model = PandasModel_pub(self.data_pub)
        self.ui.tableView_pub.setModel(self.model)
        self.model = PandasModel_echo(self.data_echo)
        self.ui.tableView_echo.setModel(self.model)

    def initUi(self):
        self.status = self.statusBar()
        self.status.showMessage('Update State', 0) #状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
        self.status.setStyleSheet("font-size: 18px;background-color: #F5E8FF")
        self.safetyNum = QtWidgets.QLabel("Safety:")
        self.taskNum = QtWidgets.QLabel("Task:")
        self.reloadNum = QtWidgets.QLabel("Reload:")

        self.safetyNum.setFixedWidth(200)
        self.safetyNum.setStyleSheet("font-size: 18px;border-radius: 25px;border: 1px solid black;")
        self.taskNum.setFixedWidth(200)
        self.taskNum.setStyleSheet("font-size: 18px;border-radius: 25px;border: 1px solid black;")
        self.reloadNum.setFixedWidth(200)
        self.reloadNum.setStyleSheet("font-size: 18px;border-radius: 25px;border: 1px solid black;")

        self.status.addPermanentWidget(self.safetyNum, stretch=0)
        self.status.addPermanentWidget(self.taskNum, stretch=0)
        self.status.addPermanentWidget(self.reloadNum, stretch=0)
        
    def setStyleSheet(self):
        self.ui.btn_ip_set.setStyleSheet("QPushButton" + "{" + "background-color:#0056ef;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        self.ui.btn_reset.setStyleSheet("QPushButton" + "{" + "background-color:#da7700;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        self.ui.btn_enable.setStyleSheet("QPushButton" + "{" + "background-color:#00d21a;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        self.ui.btn_disable.setStyleSheet("QPushButton" + "{" + "background-color:#b10011;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        self.ui.onBtn.setStyleSheet("QPushButton" + "{" + "background-color:#da7700;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        self.ui.offBtn.setStyleSheet("QPushButton" + "{" + "background-color:#da7700;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        
        self.ui.btn_start_program.setStyleSheet("QPushButton" + "{" + "background-color:#da7700;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        self.ui.btn_stop_program.setStyleSheet("QPushButton" + "{" + "background-color:#da7700;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        self.ui.btn_vel_set.setStyleSheet("QPushButton" + "{" + "background-color:#da7700;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        # self.ui.btn_acc_set.setStyleSheet("QPushButton" + "{" + "background-color:#da7700;\n" + "color:white;\n" + "border-color: black;" 
        #     + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        self.ui.btn_project_name_select.setStyleSheet("QPushButton" + "{" + "background-color:#da7700;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        self.ui.btn_project_name_read.setStyleSheet("QPushButton" + "{" + "background-color:#da7700;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        self.ui.btn_start_program.setStyleSheet("QPushButton" + "{" + "background-color:#0300fc;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        self.ui.btn_stop_program.setStyleSheet("QPushButton" + "{" + "background-color:#005b62;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
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
        
        if self.ui.comboBox.currentText() == "None":
            # self.ui.label_mission_case_show.setText('Choose：%s' % self.ui.comboBox.currentText())
            task_value = 0
            # self.ui_reload_program()
        elif self.ui.comboBox.currentText() == "Show 1":
            self.ui.label_mission_case_show.setText('Choose：%s' % self.ui.comboBox.currentText())
            task_value = 1
            self.ui_reload_program()
            self.ui.comboBox.setCurrentIndex(0)
        elif self.ui.comboBox.currentText() == "Show 2":
            self.ui.label_mission_case_show.setText('Choose：%s' % self.ui.comboBox.currentText())
            task_value = 3
            self.ui_reload_program()
            self.ui.comboBox.setCurrentIndex(0)
        elif self.ui.comboBox.currentText() == "Smile":
            self.ui.label_mission_case_show.setText('Choose：%s' % self.ui.comboBox.currentText())
            task_value = 2
            self.ui_reload_program()
            self.ui.comboBox.setCurrentIndex(0)
        elif self.ui.comboBox.currentText() == "Bow":
            self.ui.label_mission_case_show.setText('Choose：%s' % self.ui.comboBox.currentText())
            task_value = 7
            self.ui_reload_program()
            self.ui.comboBox.setCurrentIndex(0)
        elif self.ui.comboBox.currentText() == "Home":
            self.ui.label_mission_case_show.setText('Choose：%s' % self.ui.comboBox.currentText())
            task_value = 4
            self.ui_reload_program()
            self.ui.comboBox.setCurrentIndex(0)
        elif self.ui.comboBox.currentText() == "Stop":
            self.ui.label_mission_case_show.setText('Choose：%s' % self.ui.comboBox.currentText())
            task_value = 5
            self.ui_reload_program()
            self.ui.comboBox.setCurrentIndex(0)
        elif self.ui.comboBox.currentText() == "Show All Select":
            self.ui.label_mission_case_show.setText('Choose：%s' % self.ui.comboBox.currentText())
            task_value = 6
            self.ui_reload_program()
            self.ui.comboBox.setCurrentIndex(0)
        # elif self.ui.comboBox.currentText() == "jogging":
        #     self.ui.label_mission_case_show.setText('Choose：%s' % self.ui.comboBox.currentText())
        #     task_value = 7
        #     self.ui_reload_program()
        #     self.ui.comboBox.setCurrentIndex(0)
        self.mission_number = task_value
        

    def timeGo(self):
        self.timer.start(100)

    def timeStop(self):
        self.timer.stop()

    def timeReset(self):
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
        self.send_reset(4096)
        self.reset_error_robot()
        self.send_reset(4096)

    def enable_buttonClicked(self):
        self.send_reset_other_state(4096, 4) # reset and only reserve enable
        self.ui.btn_enable.setDisabled(True)
        self.ui.btn_disable.setDisabled(False)
        self.ui.btn_enable.setStyleSheet("QPushButton" + "{" + "background-color:#5151A2;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        self.ui.btn_disable.setStyleSheet("QPushButton" + "{" + "background-color:#b10011;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
    def disable_buttonClicked(self):
        self.disable_robot()
        self.ui.btn_enable.setDisabled(False)
        self.ui.btn_disable.setDisabled(True)
        self.ui.btn_enable.setStyleSheet("QPushButton" + "{" + "background-color:#00d21a;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        self.ui.btn_disable.setStyleSheet("QPushButton" + "{" + "background-color:#5151A2;\n" + "color:white;\n" + "border-color: black;" 
            + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
    def ui_reload_program(self):
        self.send_reset_other_state(4096, 4) # reset and only reserve enable
        self.reload_all_programs() 
        self.stop_programs() # reset before starting cmd
        while not rospy.is_shutdown():
            if self.is_task_init() == True:
                rospy.loginfo("reload_all_programs finished")
                self.ui.btn_start_program.setStyleSheet("QPushButton" + "{" + "background-color:#da7700;\n" + "color:white;\n" + "border-color: black;" 
                + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
                break

    def start_buttonClicked(self):
        register = 1024
        value = self.mission_number
        self.modclient.setOutput(register,value,0)
        # for show
        register = 1024 + 12 # 24
        value = 0
        self.send_16bit_value(register,value) # set SMOGetU16(24)

        start_status = self.start_programs(0)
        # self.mission_number = 0 # init
        self.ui.btn_start_program.setStyleSheet("QPushButton" + "{" + "background-color:#0300fc;\n" + "color:white;\n" + "border-color: black;" 
                + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        

    def stop_buttonClicked(self):
        self.stop_programs()

    def onBtn(self, event):
        self.topic_callback_init()

        self.thread1=MyThread(1, 100) #cycle 100ms
        self.thread1.callback.connect(self.drawUi)
        self.thread1.start()
        
        # self.thread2=MyThread(2, 100) #cycle 100ms
        # self.thread2.callback.connect(self.drawUi)
        # self.thread2.start()

        # self.thread3=MyThread(3, 100)
        # self.thread3.callback.connect(self.drawUi)
        # self.thread3.start()
        self.startthreadflag = True

        self.ui.onBtn.setDisabled(True)
        self.ui.offBtn.setDisabled(False)
        self.ui.onBtn.setStyleSheet("QPushButton" + "{" + "background-color:#5151A2;\n" + "color:white;\n" + "border-color: black;" 
                + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        self.ui.offBtn.setStyleSheet("QPushButton" + "{" + "background-color:#da7700;\n" + "color:white;\n" + "border-color: black;" 
                + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
        QMessageBox.about(self, "對話框", "開始讀取手臂資訊")

    def offBtn(self, event):
        if self.startthreadflag == True:
            self.stopThread(self.thread1)
            self.ui.offBtn.setDisabled(True)
            self.ui.onBtn.setDisabled(False)
            self.ui.onBtn.setStyleSheet("QPushButton" + "{" + "background-color:#da7700;\n" + "color:white;\n" + "border-color: black;" 
                + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
            self.ui.offBtn.setStyleSheet("QPushButton" + "{" + "background-color:#5151A2;\n" + "color:white;\n" + "border-color: black;" 
                + "}" + "QPushButton::pressed" + "{" + "background-color :#5151A2;\n" + "color:white;" +"}")
            # self.stopThread(self.thread2)
        else:
            pass

    def stopThread(self, name_or_qthread):
        """
        Stop a running thread.
        :type name_or_qthread: T <= str | QThread
        """
        try:
            qthread = name_or_qthread
            qthread.quit()
            if not qthread.wait(2000):
                qthread.terminate()
                qthread.wait()
        except Exception as e:
            rospy.loginfo('Thread shutdown could not be completed: %s', e)
        del name_or_qthread

    def ip_setClicked(self):
        text = self.ui.lineEdit_ip.text()
        self.ui.label_ip.setText("Set IP:"+text)
        self.ui.lineEdit_ip.clear()
        reply = self.ip_check(text)
        if reply == True:
            self.ip_set(text)
            self.pub_ipset.publish(text)
            rospy.loginfo("IP reset complete")
            QMessageBox.about(self, "對話框", "已設定IP, 請確認modbus server是否正常開啟")
        else:
            rospy.loginfo("Setup fail")
            QMessageBox.warning(self, "警告", "IP輸入格式錯誤, 請重新輸入")

    def ip_check(self, ip):
        sc=ip.strip().split('.') 
        if len(sc)!= 4: 
            print( "check ip address failed!")
            return False
        for i in range(4):
            b=len(sc[i])
            c=sc[i]
            for j in range(b):
                if c[j]==" ":
                    print ("check ip address failed!")
                    return False
            j+=1  
            try:
                sc[i]=int(sc[i]) 
            except: 
                print ("check ip address failed!")
                return False
            if sc[i]<=255 & sc[i]>=0:  
                pass
            else:
                print ("check ip address failed!")
                return False
            i+=1
        else: 
            print ("check ip address success!")
            return True

    def drawUi(self, index, label):
        if label == 1:
            self.safetyNum.setText("Safety: "+self.safety_state())
            self.taskNum.setText("Task: "+self.task_state(0))
            self.reloadNum.setText("Reload: "+str(self.is_task_init()))
            self.safetyNum.setStyleSheet("background-color:green;font-size: 18px;border-radius: 25px;border: 1px solid black;")
            if self.safety_state() == "Error":
                self.safetyNum.setStyleSheet("background-color:red;font-size: 18px;border-radius: 25px;border: 1px solid black;")
        # elif label == 2:
            ACS_actual = self.read_ACS_actual_position()
            ACS_command = self.read_ACS_command_position()
            PCS_actual = self.read_PCS_actual_position()
            PCS_command = self.read_PCS_command_position()
            
            self.joint_state.position = [ACS_actual.axis1, ACS_actual.axis2, ACS_actual.axis3, ACS_actual.axis4, ACS_actual.axis5, ACS_actual.axis6]
            self.pub_joint_state.publish(self.joint_state)
        
            data = np.array([
                [round(ACS_command.axis1,3),round(ACS_command.axis2,3),round(ACS_command.axis3,3),round(ACS_command.axis4,3),round(ACS_command.axis5,3),round(ACS_command.axis6,3)],
                [round(ACS_actual.axis1,3),round(ACS_actual.axis2,3),round(ACS_actual.axis3,3),round(ACS_actual.axis4,3),round(ACS_actual.axis5,3),round(ACS_actual.axis6,3)],
                ["Deg","Deg","Deg","Deg","Deg","Deg"]
                ])
            self.model = PandasModel_joint(data)
            self.ui.tableView_joint.setModel(self.model)
            self.ui.tableView_joint.update()
            data = np.array([
                [round(PCS_command.X,3),round(PCS_command.Y,3),round(PCS_command.Z,3),round(PCS_command.A,3),round(PCS_command.B,3), round(PCS_command.C,3)],
                [round(PCS_actual.X,3),round(PCS_actual.Y,3),round(PCS_actual.Z,3),round(PCS_actual.A,3),round(PCS_actual.B,3), round(PCS_actual.C,3)],
                ["mm","mm","mm","Deg","Deg","Deg"]
                ])
            self.model = PandasModel_pos(data)
            self.ui.tableView_pos.setModel(self.model)
            self.ui.tableView_pos.update()

            if self.checkflag == True and self.safety_state() == "Enable":
                self.checkflag = False
                rospy.sleep(0.5)
                ACS_actual = self.read_ACS_actual_position()
                ACS_command = self.read_ACS_command_position()
                a1 = np.array([round(ACS_command.axis1,3),round(ACS_command.axis2,3),round(ACS_command.axis3,3),round(ACS_command.axis4,3),round(ACS_command.axis5,3),round(ACS_command.axis6,3)])
                a2 = np.array([round(ACS_actual.axis1,3),round(ACS_actual.axis2,3),round(ACS_actual.axis3,3),round(ACS_actual.axis4,3),round(ACS_actual.axis5,3),round(ACS_actual.axis6,3)])
                if np.allclose(a1,a2, rtol=0.1) == False:
                    reply = QMessageBox.warning(self, "警告", "繼續執行會導致手臂非預期運動,你確定要繼續?", QMessageBox.Yes | QMessageBox.No)
                    if reply == QMessageBox.Yes:
                        reply = QMessageBox.critical(self, "Error!", "請勿讓手臂執行任何動作", QMessageBox.Yes | QMessageBox.No,)
                        if reply == QMessageBox.No:
                            QMessageBox.about(self, "提示", "將強制關閉程式...")
                            sys.exit(app.exec_())
                        else:
                            QMessageBox.about(self, "提示", "請至TPUI重新設定Home點校正")
                    else:
                        QMessageBox.about(self, "提示", "請至TPUI重新設定Home點校正")
                else:
                    QMessageBox.about(self, "提示", "手臂當前角度正確, 可正常運行")
        # else:
            self.data_pub = np.insert(self.data_pub,0,values=[[self.task_cmd,self.statusID]],axis=0)
            self.model = PandasModel_pub(self.data_pub)
            self.ui.tableView_pub.setModel(self.model)
            self.ui.tableView_pub.update()

            self.data_echo = np.insert(self.data_echo,0,values=[[self.task_cmd_reply,self.statusID_reply]],axis=0)
            self.model = PandasModel_echo(self.data_echo)
            self.ui.tableView_echo.setModel(self.model)
            self.ui.tableView_echo.update()

            
        
    def vel_setClicked(self):
        """
            set TPUI SMO 1 
        """
        register = 1024 + 4 # 8 
        self.vel = float(self.ui.lineEdit_vel.text())
        self.send_64bit_value(register,self.vel) # set SMOGetF64(8)
    # def acc_setClicked(self):
    #     """
    #         set TPUI SMO 2
    #     """
    #     register = 1024 + 8 # 16
    #     self.acc = float(self.ui.lineEdit_acc.text())
    #     self.send_64bit_value(register,self.acc) # set SMOGetF64(16)

    def VelSliderValue(self):
        self.ui.lineEdit_vel.setText(str(self.ui.horizontalSlider_vel.value()))
        
    # def AccSliderValue(self):
    #     self.ui.lineEdit_acc.setText(str(self.ui.horizontalSlider_acc.value()))
        
    def vel_slow_setClicked(self):
        """
            set TPUI SMO 1 
        """
        register = 1024 + 4 # 8 
        self.vel = 50
        self.send_64bit_value(register,self.vel) # set SMOGetF64(8)

    def vel_quick_setClicked(self):
        """
            set TPUI SMO 1 
        """
        register = 1024 + 4 # 8 
        self.vel = 100
        self.send_64bit_value(register,self.vel) # set SMOGetF64(8)

    # TODO: test button set project name   
    def project_name_setClicked(self):
        # self.project_name("modbus_test")
        pass
    # TODO: test button read project name
    def project_name_readClicked(self):
        self.ui.label_project_name.setText('Project name: %s' % self.read_project_name())
    # test button sent pcs & acs position for arm motion
    def testClicked(self):
        self.set_pcs_position(260,-112,412,-90,0,-180) # set SMOGetCP(64);
        self.set_acs_position(0.552,-68.967,-103.583,-97.449,90,0.552) # set SMOGetAP(112);
    # test button read SMI
    # def test2Clicked(self):
    #     input_registers = self.modclient.read_input_Registers(1024,1)
    #     print(input_registers)

    def closeEvent(self, event):
        reply = QMessageBox.question(self, 'Message', 'You sure to quit?',
                                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            reason = "You sure to quit"
            self.pub_closenode.publish(1)
            rospy.signal_shutdown(reason)
            event.accept()
        else:
            event.ignore()
    # message show example
    def showMsg(self):
        QMessageBox.information(self, '信息...',QMessageBox.Yes | QMessageBox.No,QMessageBox.Yes)
        QMessageBox.question(self, "提問...", QMessageBox.Yes | QMessageBox.No)
        QMessageBox.warning(self, "警告....", QMessageBox.Yes | QMessageBox.No)
        QMessageBox.critical(self, "錯誤...", QMessageBox.Yes | QMessageBox.No,)
        QMessageBox.about(self, "關於...", "......")

    # topic callback initial
    def topic_callback_init(self):
        self.pub_armstatus = rospy.Subscriber("/reply_external_comm",peripheralCmd,self.topic_reply_callback)
        self.sub_taskcmd = rospy.Subscriber("/write_external_comm",peripheralCmd,self.topic_write_callback)
        self.peripheralCmd = peripheralCmd()

    def topic_write_callback(self,data):
        self.task_cmd = data.actionTypeID
        self.statusID = data.statusID

    def topic_reply_callback(self,data):
        self.task_cmd_reply = data.actionTypeID
        self.statusID_reply = data.statusID

    # TODO：for show
    def show_program_readClicked(self):
        register = 1024 + 12 # 24
        value = 1
        self.send_16bit_value(register,value) # set SMOGetU16(24)

    def smile_program_readClicked(self):
        register = 1024 + 12 # 24
        value = 2
        self.send_16bit_value(register,value) # set SMOGetU16(24)

    def dance_program_readClicked(self):
        register = 1024 + 12 # 24
        value = 3
        self.send_16bit_value(register,value) # set SMOGetU16(24)

    def bow_program_readClicked(self):
        register = 1024 + 12 # 24
        value = 4
        self.send_16bit_value(register,value) # set SMOGetU16(24)

    def stopshow_program_readClicked(self):
        register = 1024 + 12 # 24
        value = 0
        self.send_16bit_value(register,value) # set SMOGetU16(24)

    def init_program_readClicked(self):
        register = 1024 + 12 # 24
        value = 5
        self.send_16bit_value(register,value) # set SMOGetU16(24)
    
    def home_program_readClicked(self):
        register = 1024 + 12 # 24
        value = 6
        self.send_16bit_value(register,value) # set SMOGetU16(24)

    def JogMode_display(self):
        if self.ui.comboBox_jogging_mode.currentText() == "None":
            self.jogging_postion_rel = 0
            self.jogging_posture_rel = 0
        elif self.ui.comboBox_jogging_mode.currentText() == "100 mm / 10o":
            self.jogging_postion_rel = 100
            self.jogging_posture_rel = 10
        elif self.ui.comboBox_jogging_mode.currentText() == "10 mm / 3o":
            self.jogging_postion_rel = 10
            self.jogging_posture_rel = 3
        elif self.ui.comboBox_jogging_mode.currentText() == "1 mm / 1o":
            self.jogging_postion_rel = 1
            self.jogging_posture_rel = 1
        elif self.ui.comboBox_jogging_mode.currentText() == "0.1 mm / 0.005o":
            self.jogging_postion_rel = 0.1
            self.jogging_posture_rel = 0.005

    def pose_setClicked(self, type):
        PCS_actual = self.read_PCS_actual_position()
        for case in switch(type):
            if case("+x"):
                self.set_pcs_position(round(PCS_actual.X + self.jogging_postion_rel,3),round(PCS_actual.Y,3),round(PCS_actual.Z,3),round(PCS_actual.A,3),round(PCS_actual.B,3),round(PCS_actual.C,3)) # set SMOGetCP(64);
                break
            if case("-x"):
                self.set_pcs_position(round(PCS_actual.X - self.jogging_postion_rel,3),round(PCS_actual.Y,3),round(PCS_actual.Z,3),round(PCS_actual.A,3),round(PCS_actual.B,3),round(PCS_actual.C,3)) # set SMOGetCP(64);
                break
            if case("+y"):
                self.set_pcs_position(round(PCS_actual.X,3),round(PCS_actual.Y + self.jogging_postion_rel,3),round(PCS_actual.Z,3),round(PCS_actual.A,3),round(PCS_actual.B,3),round(PCS_actual.C,3)) # set SMOGetCP(64);
                break
            if case("-y"):
                self.set_pcs_position(round(PCS_actual.X,3),round(PCS_actual.Y - self.jogging_postion_rel,3),round(PCS_actual.Z,3),round(PCS_actual.A,3),round(PCS_actual.B,3),round(PCS_actual.C,3)) # set SMOGetCP(64);
                break
            if case("+z"):
                self.set_pcs_position(round(PCS_actual.X,3),round(PCS_actual.Y,3),round(PCS_actual.Z + self.jogging_postion_rel,3),round(PCS_actual.A,3),round(PCS_actual.B,3),round(PCS_actual.C,3)) # set SMOGetCP(64);
                break
            if case("-z"):
                self.set_pcs_position(round(PCS_actual.X,3),round(PCS_actual.Y,3),round(PCS_actual.Z - self.jogging_postion_rel,3),round(PCS_actual.A,3),round(PCS_actual.B,3),round(PCS_actual.C,3)) # set SMOGetCP(64);
                break
            if case("+a"):
                self.set_pcs_position(round(PCS_actual.X,3),round(PCS_actual.Y,3),round(PCS_actual.Z,3),round(PCS_actual.A + self.jogging_posture_rel,3),round(PCS_actual.B,3),round(PCS_actual.C,3)) # set SMOGetCP(64);
                break
            if case("-a"):
                self.set_pcs_position(round(PCS_actual.X,3),round(PCS_actual.Y,3),round(PCS_actual.Z,3),round(PCS_actual.A - self.jogging_posture_rel,3),round(PCS_actual.B,3),round(PCS_actual.C,3)) # set SMOGetCP(64);
                break
            if case("+b"):
                self.set_pcs_position(round(PCS_actual.X,3),round(PCS_actual.Y,3),round(PCS_actual.Z,3),round(PCS_actual.A,3),round(PCS_actual.B + self.jogging_posture_rel,3),round(PCS_actual.C,3)) # set SMOGetCP(64);
                break
            if case("-b"):
                self.set_pcs_position(round(PCS_actual.X,3),round(PCS_actual.Y,3),round(PCS_actual.Z,3),round(PCS_actual.A,3),round(PCS_actual.B - self.jogging_posture_rel,3),round(PCS_actual.C,3)) # set SMOGetCP(64);
                break
            if case("+c"):
                self.set_pcs_position(round(PCS_actual.X,3),round(PCS_actual.Y,3),round(PCS_actual.Z,3),round(PCS_actual.A,3),round(PCS_actual.B,3),round(PCS_actual.C + self.jogging_posture_rel,3)) # set SMOGetCP(64);
                break
            if case("-c"):
                self.set_pcs_position(round(PCS_actual.X,3),round(PCS_actual.Y,3),round(PCS_actual.Z,3),round(PCS_actual.A,3),round(PCS_actual.B,3),round(PCS_actual.C - self.jogging_posture_rel,3)) # set SMOGetCP(64);
                break
            if case(): # default
                break

if __name__=="__main__":
    rospy.init_node("arm_control_ui")
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
    