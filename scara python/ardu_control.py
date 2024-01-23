#!/usr/bin/python3
import time
import serial
from numpy import *

# links
L1 = 228  # L1 = 228mm
L2 = 316 #old->291 # L2 = 136.5mm L3 = 154

camToJ2base = 477
J2baseToEnd = 366

# *********** WIP **********
# limit setting ( the max or min movement of the links and z movement)
# accurate z movement using dc motor and encoders
# dcv problems: check dcvoff value of both dcv controls

class arm_control:
    def __init__(self,serialport='/dev/ttyACM0'):
        self.ser_port = serial.Serial(port = serialport, baudrate=115200, timeout=3)
        self.wait_ser()
        self.wait_homing()

        # variables
        self.j1Slider = 0
        self.j2Slider = 0
        self.zSlider = 450
        self.saveStatus = 0
        self.flip = 0
        self.dcvValue = 0
        self.servoValue = 90              # Servo value -> 0 to 180 ; > initially at 90
        self.speedSlider = 1000
        self.accelerationSlider = 1000
        self.data = ""

        self.theta1 = 0
        self.theta2 = 0
        self.z = 0  


    def wait_ser(self): # serial confirmation
        while (self.ser_port.in_waiting == 0):
            continue
        data_r = self.ser_port.readline().decode()
        print(data_r) 
    
    def wait_homing(self): # wait untill homing is done
        while (self.ser_port.in_waiting == 0):
            continue
        data_r = self.ser_port.readline().decode()
        if data_r == "Homed": # checking for homing 
            print(data_r)            

    def write_wait(self): # wait's and returns the data
        self.ser_port.write(self.data.encode()) # writing data 
        while (self.ser_port.in_waiting == 0):
            continue
        data_r = self.ser_port.readline().decode()
        if data_r == self.data: # print confirmation
            print(data_r)


    def forwardKinematics(self):
        theta1F = self.theta1 * pi / 180   # degrees to radians
        theta2F = self.theta2 * pi / 180
        xP = round(L1 * cos(theta1F) + L2 * cos(theta1F + theta2F))
        yP = round(L1 * sin(theta1F) + L2 * sin(theta1F + theta2F))
        # Current end-effector point is (xP,yP)
        return (xP,yP)


    # Inverse Kinematics
    def inverseKinematics(self,x,y):

        r1 = sqrt(x**2+y**2)  # eqn 1
        phi_1 = arccos((L2**2 - L1**2 - r1**2) / (-2 * L1 * r1))  # eqn 2
        phi_2 = arctan2(y, x)  # eqn 3
        self.theta1 = rad2deg(phi_2-phi_1)  # eqn 4 converted to degrees

        phi_3 = arccos((r1**2 - L1**2 - L2**2) / (-2 * L1 * L2))
        self.theta2 = 180-rad2deg(phi_3)

        # added (28.11.2023)
        # Adjust angles for the fourth quadrant
        if x <= 0 and y <= 0:  # Check if in the fourth quadrant
            self.theta1 = degrees(phi_2 + phi_1)   # Opposite angle for theta1
            self.theta2 = - self.theta2  # Opposite angle for theta2 

        if x == 200 and y == 200:
            self.theta1 = degrees(phi_2 + phi_1)   # Opposite angle for theta1
            self.theta2 = - self.theta2  # Opposite angle for theta2 


        return self.theta1,self.theta2
    

    # Serial write data
    def updateData(self):
        self.data = str(self.saveStatus)+","+str(self.flip)+","+str(self.j1Slider)+","+str(self.j2Slider)+","+str(self.dcvValue)+","+str(self.zSlider)+","+str(self.servoValue)+","+str(self.speedSlider)+","+str(self.accelerationSlider)
        # Data:
            # data[0] -> save(1) ; clear(2) ; homing(3)
            # data[1] -> Flipping start -> 1
            # data[2] -> Joint 1 angle
            # data[3] -> Joint 2 angle
            # data[4] -> suction dcv value ON(1) 
            # data[5] -> Z position [ 1->upwards(4000) ; 2->downwards(limitswitch) ; 3->downwards(drop area) ; 0->no movement]
            # data[6] -> Servo value
            # data[7] -> Speed value
            # data[8] -> Acceleration value
        #return self.data 


    # Homing ***** Mostly not used *****
    def homing(self):
        self.saveStatus = 3
        self.updateData()
        self.write_wait()


    # Move to (x,y) coordinate
    def moveTo(self,x,y):
        T1,T2 = self.inverseKinematics(x,y)
        self.j1Slider = round(T1)
        self.j2Slider = round(T2)
        # return self.updateData()
        self.updateData()  # stores updated movement in self.data variable
        self.write_wait()  # write and wait's till confirmation
        

    # package drop point
    def moveToDropZone(self): 
        T1,T2 = self.inverseKinematics(50,500)   # Drop_zone coordintate is: (-150,400)
        self.j1Slider = round(T1)
        self.j2Slider = round(T2)
        # return self.updateData()
        self.updateData()
        self.write_wait()


    # package flip point (***WIP***)
    def moveToFlipZone(self):
        T1,T2 = self.inverseKinematics(-350,350)   # flip_zone coordintate is: (-150,400)
        self.j1Slider = round(T1)
        self.j2Slider = round(T2)
        # return self.updateData()
        self.updateData()
        self.write_wait()

            
    # Z movement (not used)
    def moveToZ(self,z):
        self.zSlider = z
        self.updateData()
        self.write_wait() 
    def moveTo450(self):
        self.zSlider = 450
        self.updateData()
        self.write_wait()
    def moveTo250(self):
        self.zSlider = 250
        self.updateData()
        self.write_wait()
    
    # Z-Movement
    # [ 1->upwards(4000) ; 2->downwards(limitswitch) ; 3->downwards(drop area) ; 0->no movement]
    def Zmoveup(self):
        self.zSlider = 1
        self.updateData()
        self.write_wait()
        self.zSlider = 0
        self.updateData()
    def Zmovedown(self):
        self.zSlider = 2
        self.updateData()
        self.write_wait()
        self.zSlider = 0
        self.updateData()
    def Zmovedown_dropzone(self):
        self.zSlider = 3
        self.updateData()
        self.write_wait()
        self.zSlider = 0
        self.updateData()
    def Zstop(self):
        self.zSlider = 0
        self.updateData()
        self.write_wait()


    # ********* WIP *********
    # Z-axis movement calculation using depth
    def zMovement(self,camDepth):  # 200 steps per mm  # camDepth is in meters
        camDepth = round(camDepth * 1000)
        J2baseTopoint = camDepth - camToJ2base
        EndTOpoint = J2baseTopoint - J2baseToEnd
        self.zSlider = EndTOpoint   # in mm 
        self.updateData()
        self.write_wait()


    # Suction cup DCV control:
    def dcvON(self):
        self.dcvValue = 1
        self.updateData()
        self.write_wait()  
    def dcvOFF(self):
        self.dcvValue = 0
        self.updateData()
        self.write_wait()

    
    # Flip DCV contol:
    def startFlip(self):
        self.flip = 1   # first starts flip process
        self.updateData()
        self.write_wait()
        self.flip = 0   # next stops flip process
        self.updateData()

    # Servo control(write angle for orientation):
    def servoSetAngle(self,angle):
        self.servoValue = angle
        self.updateData()
        self.write_wait()

    # Exit Connection:
    def arduStop(self):
        self.ser_port.close()
        print("Serial Connection Stopped")