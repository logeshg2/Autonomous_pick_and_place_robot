# This script contains the camera control, detection, commands to arduino for flipping mechanism

import cv2
vid = cv2.VideoCapture(0)

def start_cam():
    while True:
        _,frame = vid.read()
        cv2.imshow("Name",frame)
        k = cv2.waitKey(1)
        if k == ord('q'):
            break

def stop_cam():
    vid.release()
    cv2.destroyAllWindows()

    