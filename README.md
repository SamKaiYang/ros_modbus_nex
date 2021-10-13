
Modbus control the TECO robotic arm task package
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

### Test echo arm status (Topic)
```bash
rostopic echo /reply_external_comm
```

### Test pub task arm task command (Topic)
```bash
rostopic pub /write_external_comm modbus/peripheralCmd "actionTypeID: 4"
```

### Publish command list:
```If you want to issue a command to control the TECO robotic arm, you must call the task command.```

1. Command 1 : Start Task Program <br>
```rostopic pub /write_external_comm modbus/peripheralCmd "actionTypeID: 1"``` 
2. Command 2 : Enable Robot <br>
```rostopic pub /write_external_comm modbus/peripheralCmd "actionTypeID: 2"```
3. Command 3 : Disable Robot <br>
```rostopic pub /write_external_comm modbus/peripheralCmd "actionTypeID: 3"```
4. Command 4 : Reset Holding Register-External Control Address 4096 <br>
```rostopic pub /write_external_comm modbus/peripheralCmd "actionTypeID: 4"```
