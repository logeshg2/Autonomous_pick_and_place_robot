import cv2
import cv2.aruco as aruco
import numpy as np

def findArucoMarkers(img,draw=True):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
	
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    bboxs, ids, rejected = detector.detectMarkers(gray)
    boxes = np.array(bboxs)
    lst = boxes.tolist()
    if bboxs:
        aruco_peri = cv2.arcLength(bboxs[0],True)
        mm_per_pixel = round(160 / aruco_peri,2)
    # print(ids) -> id = [2]
    if draw:
        aruco.drawDetectedMarkers(img, bboxs)
        if lst != [] and ids == [2]:
            p1,p2 = lst[0][0][0],lst[0][0][2]
            Acx,Acy = (int(p1[0])+int(p2[0]))//2,(int(p1[1])+int(p2[1]))//2
            cv2.circle(img,(Acx,Acy),1,(0,0,255),2)
            return (Acx,Acy),mm_per_pixel
