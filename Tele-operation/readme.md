Implemented a tele-operation system for the scara robotic arm.

Features:
    1. Low-cost tele-operation system using ROS 1 and arduino microcontroller.
    2. Using the tele-operation different tasks can be taught to the robot via tele-operation.
    3. Potentiometer are used to read the values of master arm.

Usage:
    1. Build a tele-operator as given in diagram.
    2. Install ROS 1 in Ubuntu 20.04 (Note: Tested in ROS 1 Noetic and Ubuntu 20.04 Operating System)
    3. Load the given arduino script into both the respective arduino.
    4. Run "roscore"
    5. Run "rosrun rosserial_arduino serial_node.py /dev/ttyACM0 __name:=master"
    6. Run "rosrun rosserial_arduino serial_node.py /dev/ttyACM1 __name:=puppet"