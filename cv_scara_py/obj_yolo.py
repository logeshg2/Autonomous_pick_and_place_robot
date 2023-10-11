import cv2
import supervision as sv
from ultralytics import YOLO
from realsense_depth import *

dc = DepthCamera()
def main():
    model = YOLO("./runs/detect/train17/weights/best.pt")
    #model = YOLO("yolov8s.pt")
    min_depth = 12

    while True:
        ret, depth_frame, color_frame,depth_raw = dc.get_frame()
        for result in model.predict(source=color_frame,stream=True,device=0):
            frame = result.orig_img
            #print(list(result.boxes.xyxy[0]))
                    
            for box in result.boxes:
                b = box.xyxy[0]
                conf=round(float(box.conf),2)
                
                if conf >=0.5:
                    x1,y1,x2,y2=b
                    (cx,cy) = (int((x1+x2)/2),int((y1+y2)/2))
                    depth_cxcy = depth_raw.get_distance(cx,cy)
                    current_depth=depth_cxcy

                    if current_depth < min_depth:
                        #min_depth = current_depth
                        cv2.putText(frame,"{}".format(round(depth_cxcy,2)),(cx+10,cy+10),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
                        cv2.rectangle(frame,(int(b[0]),int(b[1])),(int(b[2]),int(b[3])) , (255,0,0), 2)
                        cv2.circle(frame,(cx,cy),2,(0,255,0),3)
                    else:
                        cv2.putText(frame,"{}".format(round(depth_cxcy,2)),(cx+10,cy+10),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
                        cv2.rectangle(frame,(int(b[0]),int(b[1])),(int(b[2]),int(b[3])) , (255,0,255), 2)
                        cv2.circle(frame,(cx,cy),2,(0,255,0),3)

                else:
                    continue
        cv2.imshow("Yolov8",frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
            

if __name__ == "__main__":
    main()
