## Implemented a tele-operation system for the scara robotic arm

### Features:
1. Low-cost tele-operation system using ROS 1 and arduino microcontroller.
2. Using the tele-operation different tasks can be taught to the robot via tele-operation.
3. Potentiometer are used to read the values of master arm.
4. ROS Serial communication takes place between the two arduino boards where joint state values are published and subscribed by the arms.
<img src="https://github.com/logeshg2/Autonomous_pick_and_place_robot/assets/121447333/e3f86969-3536-40c6-a88b-824abfa4dd32" width="80%"/>


### Usage:
1. Build a tele-operator as shown in image.
2. Install ROS 1 in Ubuntu 20.04  (**Note: Tested in ROS 1 Noetic and Ubuntu 20.04 Operating System**)
3. Load the given arduino script into both the respective arduino.
4. Run ```roscore```
5. Run ```rosrun rosserial_arduino serial_node.py /dev/ttyACM0 __name:=master```
6. Run ```rosrun rosserial_arduino serial_node.py /dev/ttyACM1 __name:=puppet```

### Tele-operation:
<img src="https://github.com/logeshg2/Autonomous_pick_and_place_robot/blob/main/Images/telrop.jpg" width="50%"/>

### Potentiometer:
<img src="https://github.com/logeshg2/Autonomous_pick_and_place_robot/blob/main/Images/potent.jpg" width="50%"/>
