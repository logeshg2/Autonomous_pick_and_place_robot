import cv2
import serial
from time import sleep
from ultralytics import YOLO
from ardu_serial import *
from realsense_depth import *
from obj_yolo import *
from aruco import *

# Steps:
# 1. Run main.py and turn ON the robot (on the camera to find the first box or flyer)
# 2. homing() of the robot
# 3. set the user frame
# 4. ON the camera and take a single picture and pass it to obj_yolo.py for for detection of boxes and aruco code
# 5. find the coordinates of the box with respect to aruco code 
# 6. get the depth of box for z-axis
# 7. pass the depth and coordinates to ardu_serial.py where inversekinematics is applied
# 8. from ardu_serial.py arduino gets serial input as joint angles
# 9. movement of robot and suction ON
# 10. after picking the robot moves to "pack_drop()" in ardu_serail.py nad drop the package by suction OFF.
# 11. Loop stops when there is no detection of boxes in the picture.
# 12. after loop homing() is done.

# Serial connection with arduino
ardu_serial = serial.Serial(port="/dev/ttyUSB0",   baudrate=115200, timeout=3)
ardu_serial.dtr = False
sleep(2)

# Flags
inDropZone = False
alldone = False
AruCxCy = None

def main():
    global j1Slider,j2Slider,zSlider,saveStatus,runStatus,dcvValue,gripperValue,speedSlider,accelerationSlider
    global inDropZone,ardu_serial

    while (ardu_serial.in_waiting == 0):
        pass
    data_r = ardu_serial.readline().decode()
    print(data_r) # serial confirmation

    # Homing
    while (ardu_serial.in_waiting == 0):
        pass
    data_r = ardu_serial.readline().decode()
    if data_r == "Homed": #checking for homing
        print(data_r)

    data = pack_drop()  # initially movement to the drop zone
    ardu_serial.write(data.encode())
    while (ardu_serial.in_waiting == 0):
        pass
    data_r = ardu_serial.readline().decode()
    if data_r == data:
        print(data_r)
    time.sleep(2)
    print("Reached drop zone")
    time.sleep(3)
    inDropZone = True

    '''# Movement (J1 and J2):
    data = inverseKinematics(163,-163)
    #data = "0,0,123,23,0,180,500,500"
    print("Here") 
    ardu_serial.write(data.encode())
    time.sleep(5)
    print("Writen")
    while (ardu_serial.in_waiting == 0):
        pass
    data_r = ardu_serial.readline().decode()
    print(data_r)
    if data_r == data:
        print(data_r)
    dest_coord = (163,-163)    
    time.sleep(3)          # waiting for the robot to move 
    print(f"Reached the point: {dest_coord}")
    time.sleep(2)
    inDropZone = False'''

    # loading the yolo-model
    model = YOLO("./yolo_weights/best.pt")
    if inDropZone:
        # Main loop to perform Pick and Place operation:
        while True:       #Note: Stops when there is no detection inside the user-frame
            # depth camera
            dc = DepthCamera()
            
            ret, depth_frame, color_frame, depth_raw = dc.get_frame()

            if inDropZone == True: # detection takes place when the robot is in drop zone
                # Package Detection 
                for result in model.predict(source=color_frame,stream=True,device=0): 
                    frame = result.orig_img
                    #print(list(result.boxes.xyxy[0]))
                    listConf = [] 
                    listCls = []
                    listDepth = []
                    listcxcy = []
                    for box in result.boxes:
                        print("inside")
                        b = box.xyxy[0]
                        conf=round(float(box.conf),2)
                        cls = box.cls
                        #print(cls.tolist())
                        if conf >=0.7:
                            listConf.append(conf)
                            listCls.append(cls)
                            x1,y1,x2,y2=b
                            (cx,cy) = (int((x1+x2)/2),int((y1+y2)/2))  # (cx,cy) is the centre point of object
                            depth_cxcy = depth_raw.get_distance(cx,cy)  # depth from the camera
                            listcxcy.append((cx,cy))
                            listDepth.append(depth_cxcy)
                            cv2.putText(frame,"{}".format(round(depth_cxcy,2)),(cx+10,cy+10),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
                            cv2.rectangle(frame,(int(b[0]),int(b[1])),(int(b[2]),int(b[3])) , (255,0,255), 2)
                            cv2.circle(frame,(cx,cy),2,(0,255,0),3)
                    
                    # Note: Arucxcy,destxy is in pixel 
                    # aruco marker detection
                    try:
                        AruCxCy,mmperPixel = findArucoMarkers(frame,draw=True) # this position is (414,312)
                    except TypeError:
                        #dc.release()
                        AruCxCy,mmperPixel = None,None
                        print("No aruco detected")
                        continue

                # Destination coordinate and depth
                try:
                    destcxcy,destDepth = destPoint(listCls,listConf,listcxcy,listDepth)
                except ValueError:
                    continue
                if (AruCxCy and destcxcy):
                    # Distance to move from aruco to the particular point:
                    alldone = True
                    dest_coord = distMovement(AruCxCy,destcxcy,mmperPixel) # Returns None if package not inside trolley
                else:
                    alldone =False
                #print(dest_coord)

                if (destcxcy and AruCxCy):
                    # Reference (draw line between aruco and destination)
                    alldone = True
                    cv2.line(frame,(AruCxCy[0],AruCxCy[1]),(destcxcy[0],destcxcy[1]),(255,0,0),2)
                else:
                    alldone = False

            cv2.imshow("Output",frame) # Video Output
            time.sleep(5)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Stop by pressing 'q'
                cv2.destroyAllWindows()
                break
            
            if alldone: # checking if there is detection
                dc.release() # Closing the depth camera
                print("Camera off")
            else:
                continue

            #print(dest_coord)
            if dest_coord == None: # checking for valid coordinate point
                continue
            
            # Moving to coordinate
            data = moveTo(dest_coord[0],dest_coord[1])
            ardu_serial.write(data.encode("utf-8"))
            while (ardu_serial.in_waiting == 0):
                pass
            data_r = ardu_serial.readline().decode()
            if data_r == data:
                print(data_r)
            time.sleep(10)
            print(f"Reached the point: {dest_coord}")
            time.sleep(2)

            # Movement (J1 and J2):
            '''data = inverseKinematics(dest_coord[0],dest_coord[1])
            print("Inverse movement")
            ardu_serial.write(data.encode("utf-8"))
            print(data)
            print(readData())
            while (readData() != data):                 # confirmation of movement
                continue
            time.sleep(6)          # waiting for the robot to move 
            print(f"Reached the point: {dest_coord}")
            time.sleep(5)'''
            inDropZone = False

            # Depth to move(zMovement)
            data = zMovement(destDepth)
            ardu_serial.write(data.encode("utf-8"))
            while (ardu_serial.in_waiting == 0):
                pass
            data_r = ardu_serial.readline().decode()
            if data_r == data:
                print(data_r)
            time.sleep(10)
            print(f"Reached depth: {destDepth}")
            time.sleep(2)

            # Suction ON 
            data = dcvON()
            ardu_serial.write(data.encode("utf-8"))
            while (ardu_serial.in_waiting == 0):
                pass
            data_r = ardu_serial.readline().decode()
            if data_r == data:
                print(data_r)
            #time.sleep(2)
            print("Suction On")
            time.sleep(2)

            # Z-Movement to home height(450mm):
            data = moveTo450()
            ardu_serial.write(data.encode("utf-8"))
            while (ardu_serial.in_waiting == 0):
                pass
            data_r = ardu_serial.readline().decode()
            if data_r == data:
                print(data_r)
            time.sleep(3)
            print("Reached 450 height")
            time.sleep(2)

            # Moving to drop zone 
            data = pack_drop()  
            ardu_serial.write(data.encode())
            while (ardu_serial.in_waiting == 0):
                pass
            data_r = ardu_serial.readline().decode()
            if data_r == data:
                print(data_r)
            time.sleep(2)
            print("Reached drop zone")
            time.sleep(3)
            inDropZone = True

            # Drop the package (Suction OFF)
            data = dcvOFF()
            ardu_serial.write(data.encode("utf-8"))
            while (ardu_serial.in_waiting == 0):
                pass
            data_r = ardu_serial.readline().decode()
            if data_r == data:
                print(data_r)
            #time.sleep(2)
            print("Suction off")
            time.sleep(2)

    ardu_serial.close()  # closing arduino serial 

if __name__ == "__main__":
    main()