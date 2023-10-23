#!/usr/bin/python3
import serial
import time
from numpy import *
from math import sin, cos, acos, atan, pi


# (414,312)

# Arduino Serial connection
#ardu_serial = serial.Serial(port="/dev/ttyUSB4",   baudrate=115200, timeout=1)
#time.sleep(3)
#ardu_serial.setDTR(False)

# variables
j1Slider = 0
j2Slider = 0
zSlider = 450
saveStatus = 0
runStatus = 0
dcvValue = 0
gripperValue = 180
speedSlider = 1000
accelerationSlider = 1000
data = ""

L1 = 228  # L1 = 228mm
L2 = 291 # L2 = 136.5mm L3 = 154
theta1,theta2,z = 0,0,0   

camToJ2base = 477
J2baseToEnd = 366

# Z-axis movement calculation using depth
def zMovement(camDepth):  # 200 steps per mm  # camDepth is in meters
  global zSlider
  camDepth = round(camDepth * 1000)
  J2baseTopoint = camDepth - camToJ2base
  EndTOpoint = J2baseTopoint - J2baseToEnd
  zSlider = EndTOpoint   # in mm 
  return updateData()

def moveTo450():
  global zSlider
  zSlider = 450
  return updateData()

def moveTo250():
  global zSlider
  zSlider = 250
  return updateData()

# FORWARD KINEMATICS
def forwardKinematics():
  theta1F = theta1 * pi / 180   # degrees to radians
  theta2F = theta2 * pi / 180
  global xP, yP
  yP = round(L1 * cos(theta1F) + L2 * cos(theta1F + theta2F))
  xP = round(L1 * sin(theta1F) + L2 * sin(theta1F + theta2F))

def inverseKinematics(x,y):
  global j1Slider,j2Slider

  r1 = sqrt(x**2+y**2)  # eqn 1
  phi_1 = arccos((L2**2 - L1**2 - r1**2) / (-2 * L1 * r1))  # eqn 2
  phi_2 = arctan2(y, x)  # eqn 3
  theta1 = rad2deg(phi_2-phi_1)  # eqn 4 converted to degrees

  phi_3 = arccos((r1**2 - L1**2 - L2**2) / (-2 * L1 * L2))
  theta2 = 180-rad2deg(phi_3)

  # 4th quadrant:
  if (x > 0 and y < 0):
    theta1 = theta1 + 180
    theta2 = theta2 - 270

  # update
  j1Slider = round(theta1)
  j2Slider = round(theta2)

  return updateData()

# Homing
def homing():
  global saveStatus
  saveStatus = 3
  return updateData()

def inverseKinematics1(x,y):
  global j1Slider,j2Slider

  r1 = sqrt(x**2+y**2)  # eqn 1
  phi_1 = arccos((L2**2 - L1**2 - r1**2) / (-2 * L1 * r1))  # eqn 2
  phi_2 = arctan2(y, x)  # eqn 3
  theta1 = rad2deg(phi_2-phi_1)  # eqn 4 converted to degrees

  phi_3 = arccos((r1**2 - L1**2 - L2**2) / (-2 * L1 * L2))
  theta2 = 180-rad2deg(phi_3)

  return theta1,theta2

# package drop point
def pack_drop(): # it returns the theta1, theta2 angles for drop_zone coordinates
  global j1Slider,j2Slider 
  T1,T2 = inverseKinematics1(-150,400)   # Drop_zone coordintate is: (-150,400)
  #print(T1,T2)
  j1Slider = round(T1)
  j2Slider = round(T2)
  return updateData()

def moveTo(x,y):
  global j1Slider,j2Slider,speedSlider,accelerationSlider
  #speedSlider = 500
  #accelerationSlider = 500
  T1,T2 = inverseKinematics1(x,y)
  #print(T1,T2)
    
  # 4th quadrant:
  #if (x > 0 and y < 0):
  #  T1 = T1 + 180
  #  T2 = T2 - 270
  
  j1Slider = round(T1)
  j2Slider = round(T2)
  return updateData()

# DCV control:
def dcvON():
  global dcvValue
  dcvValue = 1
  return updateData()  
def dcvOFF():
  global dcvValue
  dcvValue = 0
  return updateData

# Serial write data
def updateData():
  global data
  data = str(saveStatus)+","+str(runStatus)+","+str(j1Slider)+","+str(j2Slider)+","+str(dcvValue)+","+str(zSlider)+","+str(gripperValue)+","+str(speedSlider)+","+str(accelerationSlider)
  # Data:
    # data[0] -> save(1) ; clear(2) ; homing(3)
    # data[1] -> run(1) ; stop(0)
    # data[2] -> Joint 1 angle
    # data[3] -> Joint 2 angle
    # data[4] -> suction dcv value ON(1) ; OFF(0)
    # data[5] -> Z position
    # data[6] -> Gripper value
    # data[7] -> Speed value
    # data[8] -> Acceleration value
  return data

def readData(ardu_serial):
  while True:
    cont = ardu_serial.readline().decode("utf-8")
    if cont:
      # print(cont)
      break
    #print(cont)
  return cont

'''def main():
  global j1Slider,j2Slider,zSlider,saveStatus,runStatus,dcvValue,gripperValue,speedSlider,accelerationSlider
  T1,T2 = inverseKinematics(-50,300)
  saveStatus = 0
  runStatus = 0
  j1Slider = T1 #T1
  j2Slider = T2 #T2
  dcvValue = 0
  zSlider = 0
  gripperValue = 180
  speedSlider = 500
  accelerationSlider = 500
  data = updateData()
  print(T1,T2)
  while True:
    ardu_serial.write(data.encode('utf-8'))
    content = ardu_serial.readline()
    #time.sleep(2)
    if content:
      break
  ardu_serial.close()
  print(content)'''

#main()

'''data = homing()
while True:
  ardu_serial.write(data.encode('utf-8'))
  cont = ardu_serial.readline()
  if cont:
    break
ardu_serial.close()'''

'''ardu_serial = serial.Serial(port="/dev/ttyACM5",   baudrate=115200,timeout=0.1)
data = pack_drop()
while True:
  ardu_serial.write(data.encode('utf-8'))
  cont = ardu_serial.readline()
  if cont:
    break
ardu_serial.close()'''
'''theta1 = 37
theta2 = 0
forwardKinematics()
print(xP,yP)
inverseKinematics1(365,0,0)
print(inverseKinematics2(365,0))
print(inverseKinematics3(365,0))'''

# def turnOn():
#   global ardu_serial
#   ardu_serial = serial.Serial(port="/dev/ttyUSB1",   baudrate=115200, timeout=1)
# def close():
#   global ardu_serial
#   ardu_serial.close()