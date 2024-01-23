import cv2
import numpy as np
from math import sqrt
import cv2.aruco as aruco

# ********* WIP **********
# to find the accurate distance (checking is bending)
# this function is available in detection.py 

def findArucoMarkers(frame,draw=True):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
	
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    bboxs, ids, rejected = detector.detectMarkers(gray)
    boxes = np.array(bboxs)
    lst = boxes.tolist()
    if bboxs:
        aruco_peri = cv2.arcLength(bboxs[0],True)
        mm_per_pixel = round(160 / aruco_peri,2)
    # print(ids) -> id = [2]
    if draw:
        aruco.drawDetectedMarkers(frame, bboxs)
        if lst != [] and ids == [1]:
            p1,p2 = lst[0][0][0],lst[0][0][2]
            Acx,Acy = (int(p1[0])+int(p2[0]))//2,(int(p1[1])+int(p2[1]))//2
            cv2.circle(frame,(Acx,Acy),1,(0,0,255),2)
            #cv2.circle(frame,(320,240),1,(0,0,255),2)
            #cv2.line(frame,(Acx,Acy),(320,240),(0,0,255),2)
            #dist = sqrt((320-Acx)**2 + (240-Acy)**2)
            #cv2.putText(frame,f"{dist}",(50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
            #return (Acx,Acy),mm_per_pixel

vid = cv2.VideoCapture(0)
vid.set(cv2.CAP_PROP_FRAME_WIDTH,640)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

while True:
    _,frame =vid.read()
    findArucoMarkers(frame,draw=True)
    cv2.imshow("OUT",frame)
    key = cv2.waitKey(3)
    if key == ord("q"):
        break

cv2.destroyAllWindows()
vid.release()