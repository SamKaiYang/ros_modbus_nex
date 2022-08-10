
Modbus control the TECO robotic arm task package
======
## Prerequisities

In order to have all dependencies installed, you need to install Collin's pymodbus, as well as the
twisted server implementation, his work is based on.
```bash
sudo apt-get update && sudo apt-get -y upgrade
python -m pip install --user --upgrade pip==20.2.4
sudo apt-get install python-pymodbus
sudo apt-get install python-pyasn1 python-twisted-conch
pip install pymodbus --upgrade
sudo apt-get install build-essential
sudo apt-get install qttools5-dev-tools
sudo apt-get install qtcreator
sudo apt-get install qt5-default
sudo apt-get install qt5-doc
sudo apt-get install qt5-doc-html qtbase5-doc-html
sudo apt-get install qtbase5-examples
sudo apt-get install libxcb-xinerama0
sudo pip3 install pyqt5
sudo pip install PySide2
```
## Quickstart

Start a modbus control arm task:
```bash
rosrun modbus nex_control_strategy_topic.py
```
Start GUI control arm task:
```bash
roslaunch modbus arm_control_gui.launch 
```
## ROS topic control
### Publish command list:
```If you want to issue a command to control the TECO robotic arm, you must call the task command.```

1. Command 1 : Start Simple Task Program <br>
```bash
rostopic pub /write_external_comm modbus/peripheralCmd "actionTypeID: 1"
```
2. Command 2 : Start init task <br>
```bash
rostopic pub /write_external_comm modbus/peripheralCmd "actionTypeID: 2"
```
3. Command 3 : Start Show Dance <br>
```bash
rostopic pub /write_external_comm modbus/peripheralCmd "actionTypeID: 3"
```
4. Command 4 : Start back_home_task <br>
```bash
rostopic pub /write_external_comm modbus/peripheralCmd "actionTypeID: 4"
```
5. Command 5 : Stop Task Program <br>
```bash
rostopic pub /write_external_comm modbus/peripheralCmd "actionTypeID: 5"
```
6. Command 6 : Reset Holding Register-External Control Address 4096 <br>
```bash
rostopic pub /write_external_comm modbus/peripheralCmd "actionTypeID: 6"
```

### Subscribe arm status:
```bash
rostopic echo /reply_external_comm
```
1. Arm task is running <br>
```actionTypeID: 99 statusID: 99``` 
2. Arm task is exit or idle or init <br>
```actionTypeID: 100 statusID: 100```
