import serial
from math import sin, cos, acos, atan, pi
import time

#ardu_serial = serial.Serial(port='/dev/ttyUSB0',   baudrate=115200, timeout=.1)

'''
def write_read(x):
    arduino.write(bytes(x,   'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return   data


while True:
    num = input("Enter a number: ")
    value   = write_read(num)
    print(value)
'''

# variables
j1Slider = 0
j2Slider = 0
j3Slider = 0
zSlider = 100
saveStatus = 0
runStatus = 0
gripperValue = 180
speedSlider = 500
accelerationSlider = 500
data = ""

xP=365  # xP,yP and zP are coordinates for homing()
yP=0
zP=100
L1 = 228  # L1 = 228mm
L2 = 136.5 # L2 = 136.5mm
theta1, theta2, phi, z = 1, 1, 1,1   


# FORWARD KINEMATICS
def forwardKinematics():
  theta1F = theta1 * pi / 180   # degrees to radians
  theta2F = theta2 * pi / 180
  global xP, yP
  xP = round(L1 * cos(theta1F) + L2 * cos(theta1F + theta2F))
  yP = round(L1 * sin(theta1F) + L2 * sin(theta1F + theta2F))

# INVERSE KINEMATICS
def inverseKinematics(x, y, z): 
  theta2 = acos(((x*x) + (y*y) - (L1*L1) - (L2*L2)) / (2 * L1 * L2))
  if (x < 0 & y < 0):
    theta2 = (-1) * theta2
  
  theta1 = atan(x / y) - atan((L2 * sin(theta2)) / (L1 + L2 * cos(theta2)))
  
  theta2 = (-1) * theta2 * 180 / pi
  theta1 = theta1 * 180 / pi

  # Angles adjustment depending in which quadrant the final tool coordinate x,y is
  if (x >= 0 & y >= 0):       # 1st quadrant
    theta1 = 90 - theta1

  if (x < 0 & y > 0):       # 2nd quadrant
    theta1 = 90 - theta1
  
  if (x < 0 & y < 0):       # 3d quadrant
    theta1 = 270 - theta1
    phi = 270 - theta1 - theta2
    phi = (-1) * phi

  if (x > 0 & y < 0):       # 4th quadrant
    theta1 = -90 - theta1
  
  if (x < 0 & y == 0):
    theta1 = 270 + theta1
  
  
  # Calculate "phi" angle so gripper is parallel to the X axis
  phi = 90 + theta1 + theta2
  phi = (-1) * phi

  # Angle adjustment depending in which quadrant the final tool coordinate x,y is
  if (x < 0 & y < 0):       # 3d quadrant
    phi = 270 - theta1 - theta2
  
  if (abs(phi) > 165):
    phi = 180 + phi


  theta1=round(theta1)
  theta2=round(theta2)
  phi=round(phi)
  
  global j1Slider,j2Slider,j3Slider,zSlider
  zP = z
  j1Slider = theta1
  j2Slider = theta2
  j3Slider = phi
  zSlider = zP



# User frame
def user_frame():
  xT,yT,zT = 0, 0, 0  # tool frame increment lenght for end effector (## yT = 155)
  X,Y,Z = (225 + xT),(136 + yT),(200 + zT)  # end effector point is changed 
  inverseKinematics(X,Y,Z)
  updateData()
  print(data)
  


# package drop point
def pack_drop():
  pass




# Serial write data
def updateData():
  global data
  data = f'''{saveStatus}
    +","+{runStatus}
    +","+{j1Slider}
    +","+{j2Slider}
    +","+{j3Slider}
    +","+{zSlider}
    +","+{gripperValue}
    +","+{speedSlider}
    +","+{accelerationSlider}'''
  