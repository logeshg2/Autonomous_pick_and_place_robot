import cv2
import time
import threading
import numpy as np
from math import sqrt
import cv2.aruco as aruco
from ultralytics import YOLO
from realsense_depth import *
from ultralytics.utils.plotting import Annotator 

class camera():
    def __init__(self):
        self.c_frame = self.d_frame = None
        self.new_frame = False
        # self.vid = cv2.VideoCapture(0)  # Get video frame  # webcam
        self.vid = DepthCamera()  # Depth camera instance
        # self.vid.set(cv2.CAP_PROP_FRAME_WIDTH,848)  # resize the frame
        # self.vid.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        self.model_intel = YOLO("./yolo_weights/intel.pt")
        self.model_econ = YOLO("./yolo_weights/econ.pt")
        self.model_best = YOLO("./yolo_weights/best.pt")

        # Lists:  --> **** These are detection done in one instance / frame (So - when needed in main loop) *****
        # Note: --> The index's are respestive to each other
        self.listConf = []
        self.listCls = []
        self.listDepth = []
        self.listcxcy = []

    
    def cam_feed(self):
        while True:   # loop for ever
            # check_frame,frame_ = self.vid.read()
            check_frame,depth_frame,color_frame,depth_raw = self.vid.get_frame()
            if (not check_frame):
                return
            self.c_frame = color_frame # changes the self.frame for each and every frame
            self.d_frame = depth_raw # numpy array (not sure)
            self.new_frame = True
            #time.sleep(0.05)

    def detect_frame(self):  # YOLO detection
        annotator = Annotator(self.c_frame)
        for result in self.model_best.predict(source=self.c_frame,stream=True,show=True):
            for box in result.boxes:
                b = box.xyxy[0]
                conf = round(float(box.conf),2)
                cls = box.cls
                if conf > 0.5:
                    x1,y1,x2,y2=b
                    (cx,cy) = (int((x1+x2)/2),int((y1+y2)/2))
                    self.listConf.append(conf)
                    self.listCls.append(cls)
                    self.listcxcy.append((cx,cy))   # stored as -> [(c1,c2),(c3,c4),...]
                    self.listDepth.append((self.getdepth(cx,cy)))
                    annotator.box_label(b, self.model_intel.names[int(cls)],(0,0,255),(0,255,0))
                    self.c_frame = annotator.result()

    def view(self):
        while not self.new_frame:
            time.sleep(0.1)
        while True:
            if self.new_frame:
                self.detect_frame()
                #self.findArucoMarker(draw=True)
                cv2.circle(self.c_frame,(320,240),1,(0,0,255),2)   # Centre point of the screen
                cv2.imshow("OUTPUT",self.c_frame)
                self.new_frame = False
            key = cv2.waitKey(1)
            if key == ord("q"):
                self.vid.release()
                break
    

    def getdepth(self,x,y):  # depth of point (x,y)
        depth = self.d_frame.get_distance(x,y)  # in meters(m)
        return depth
    

    def findArucoMarker(self,draw=True):  # to get centre of aruco
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        parameters =  cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        
        gray = cv2.cvtColor(self.c_frame, cv2.COLOR_BGR2GRAY)
        bboxs, ids, rejected = detector.detectMarkers(gray)
        boxes = np.array(bboxs)
        lst = boxes.tolist()
        if bboxs:
            aruco_peri = cv2.arcLength(bboxs[0],True)
            mm_per_pixel = round(160 / aruco_peri,2)
        # print(ids) -> id = [2]
        if draw:
            aruco.drawDetectedMarkers(self.c_frame, bboxs)
            if lst != [] and ids == [1]:
                p1,p2 = lst[0][0][0],lst[0][0][2]
                Acx,Acy = (int(p1[0])+int(p2[0]))//2,(int(p1[1])+int(p2[1]))//2
                cv2.circle(self.c_frame,(Acx,Acy),1,(0,0,255),2)
                # cv2.circle(self.c_frame,(320,240),1,(0,0,255),2)   # Centre point of the screen
                # cv2.line(self.c_frame,(Acx,Acy),(320,240),(0,0,255),2)
                # dist = sqrt((320-Acx)**2 + (240-Acy)**2)
                # cv2.putText(self.c_frame,f"{dist}",(50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
                return (Acx,Acy),mm_per_pixel
            else:
                return None,None


    def findDestcxcy(self,lco,lcs,lxy):
        max_of_conf = max(lco)
        index_of_maxconf = lco.index(max_of_conf)
        return lcs[index_of_maxconf],lxy[index_of_maxconf]


    # ***IMP***
    # cls-> ['bbox','bflyer','sbox','sflyer','ubox','uflyer']
    # conf-> [0.7 to 1]


    def cam2_img(self):
        vid = cv2.VideoCapture(1)
        _,img = vid.read()
        vid.release()
        return img

    def cam2_detect(self,img):
        #annotator = Annotator(img)
        lconf2=lcls2=lcxcy2=[]
        for result in self.model_econ.predict(source=img,stream=True):
            for box in result.boxes:
                b = box.xyxy[0]
                conf = round(float(box.conf),2)
                cls = box.cls
                if conf > 0.5:
                    x1,y1,x2,y2=b
                    (cx,cy) = (int((x1+x2)/2),int((y1+y2)/2))
                    lconf2.append(conf)
                    lcls2.append(cls)
                    #lcxcy2.append((cx,cy))   # stored as -> [(c1,c2),(c3,c4),...]
                    #self.listDepth.append((self.getdepth(cx,cy)))
                    #annotator.box_label(b, self.model.names[int(cls)],(0,0,255),(0,255,0))
                    #self.img = annotator.result()
                    return lconf2,lcls2
                else:
                    return None,None

# obj = camera()
# thread = threading.Thread(target=obj.cam_feed)
# thread.daemon=True
# thread.start()
# obj.view()
