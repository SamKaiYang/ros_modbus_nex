#!/usr/bin/env python
# -*-coding:utf-8 -*-
import rospy
import threading
import time
import numpy as np
from modbus.modbus_nex_api import ModbusNexApi 
from modbus.msg import peripheralCmd
from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from main_ui import Ui_MainWindow
from nex_control_strategy_topic import nex_control, switch
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

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
class MainWindow(QtWidgets.QMainWindow, ModbusNexApi, nex_control):
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
        # test 
        self.i = 0
        self.j = 0
        self.ui.setupUi(self)
        self._creat_menubar()
        self.setWindowIcon(QtGui.QIcon('../picture/teco_icon.png'))
        # self.setWindowIcon(QtGui.QIcon('/picture/teco_icon.png'))
        self.ui.lineEdit_vel.setText(str(self.vel))
        self.ui.lineEdit_acc.setText(str(self.acc))
        self.ui.btn_reset.clicked.connect(self.reset_buttonClicked)
        self.ui.btn_enable.clicked.connect(self.enable_buttonClicked)
        self.ui.btn_disable.clicked.connect(self.disable_buttonClicked)
        self.ui.btn_reload.clicked.connect(self.reload_buttonClicked)
        self.ui.onBtn.clicked.connect(self.onBtn)
        self.ui.offBtn.clicked.connect(self.offBtn)
        self.ui.btn_start_program.clicked.connect(self.start_buttonClicked)
        self.ui.btn_vel_set.clicked.connect(self.vel_setClicked)
        self.ui.btn_acc_set.clicked.connect(self.acc_setClicked)
        self.ui.btn_ip_set.clicked.connect(self.ip_setClicked)
        self.ui.btn_project_name_select.clicked.connect(self.project_name_setClicked)
        # test button 
        self.ui.btn_test.clicked.connect(self.testClicked)
        self.ui.btn_test2.clicked.connect(self.test2Clicked)
        self.ui.btn_test_pub.clicked.connect(self.testpubClicked)

        # modbus connect server ip init
        self.ip_init()
        # arm position class define
        self.point_init() 
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
        self.showMsg()

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

    def disable_buttonClicked(self):
        self.disable_robot()

    def reload_buttonClicked(self):
        self.send_reset_other_state(4096, 4) # reset and only reserve enable
        self.reload_all_programs() 
        self.stop_programs() # reset before starting cmd
        while not rospy.is_shutdown():
            if self.is_task_init() == True:
                rospy.loginfo("reload_all_programs finished")
                break

    def start_buttonClicked(self):
        register = 1024
        value = self.mission_number
        self.modclient.setOutput(register,value,0)
        start_status = self.start_programs(0)

    def onBtn(self, event):
        self.topic_callback_init()

        self.thread1=MyThread(1, 100) #cycle 100ms
        self.thread1.callback.connect(self.drawUi)
        self.thread1.start()
        
        self.thread2=MyThread(2, 100) #cycle 100ms
        self.thread2.callback.connect(self.drawUi)
        self.thread2.start()

        self.thread3=MyThread(3, 100)
        self.thread3.callback.connect(self.drawUi)
        self.thread3.start()
        

        QMessageBox.about(self, "對話框", "開始讀取手臂資訊")
    def offBtn(self, event):
        self.stopThread(self.thread1)
        self.stopThread(self.thread2)

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
            rospy.loginfo("Setup complete")
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
            if self.safety_state() == "Error":
                self.safetyNum.setStyleSheet("background-color:red;font-size: 18px;border-radius: 25px;border: 1px solid black;")
        elif label == 2:
            ACS_actual = self.read_ACS_actual_position()
            ACS_command = self.read_ACS_command_position()
            PCS_actual = self.read_PCS_actual_position()
            PCS_command = self.read_PCS_command_position()
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
                    # print(np.allclose(a1,a2, rtol=0.1))
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
        else:
            # TODO： test ros topic echo from nex_control 
            self.ui.label_rostopic_pub_show.setText("task_cmd:"+ str(self.task_cmd)  +"statusID:" + str(self.statusID))
            self.ui.label_rostopic_echo_show.setText("task_cmd:"+ str(self.task_cmd_reply)  +"statusID:" + str(self.statusID_reply))
            # add tableview function for rostopic result show
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
        register = 1025
        self.vel = int(self.ui.lineEdit_vel.text())
        self.modclient.setOutput(register,self.vel,0)

    def acc_setClicked(self):
        """
            set TPUI SMO 2
        """
        register = 1026
        self.acc = int(self.ui.lineEdit_acc.text())
        self.modclient.setOutput(register,self.acc,0)

    def VelSliderValue(self):
        self.ui.lineEdit_vel.setText(str(self.ui.horizontalSlider_vel.value()))
        
    def AccSliderValue(self):
        self.ui.lineEdit_acc.setText(str(self.ui.horizontalSlider_acc.value()))

    def project_name_setClicked(self):
        self.read_project_name()

    # test button sent pcs position
    def testClicked(self):
        self.set_pcs_position(0,-162.5,818.1,-180,0,-90)
    # test button sent acs position
    def test2Clicked(self):
        self.set_acs_position(0,-90,0,-90,0,0)
    # TODO: test  pub list show update test
    # test pub list show
    def testpubClicked(self):
        # self.data_pub = np.append(self.data_pub,[[1,2]],0)
        self.i = self.i + 1
        self.j = self.j + 1
        self.data_pub = np.insert(self.data_pub,0,values=[[self.i,self.j]],axis=0)
        self.model = PandasModel_pub(self.data_pub)
        self.ui.tableView_pub.setModel(self.model)
        # self.ui.tableView_pub.model().layoutChanged.emit()
        self.ui.tableView_pub.update()

    def closeEvent(self, event):
        reply = QMessageBox.question(self, 'Message', 'You sure to quit?',
                                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
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

if __name__=="__main__":
    rospy.init_node("arm_control_ui")
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
    