
modbus control teco robotic arm task package
======
## Prerequisities

In order to have all dependencies installed, you need to install Collin's pymodbus, as well as the
twisted server implementation, his work is based on.
```bash
sudo apt-get install python-pymodbus
sudo apt-get install python-pyasn1 python-twisted-conch
```
## Quickstart

Start a modbus control arm task:
```bash
rosrun modbus nex_control_strategy_topic.py 
```
## ROS topic control

Test echo arm status (Topic)
```bash
rostopic echo /arm_status
```

Test pub task arm task command (Topic)
```bash
rostopic pub /armTask_cmd  modbus/send "task_cmd: 1"
```
