from ardu_control import arm_control
from detection import camera
from obj_coord import *
import threading
import time

# main function:
def main():
    global inFlipZone
    ardu.moveToDropZone() # Robot waits in dropzone
    while True:
        # Step1:
        # Detection of packages
        lconf = cam.listConf
        lcls = cam.listCls
        lcxcy = cam.listcxcy
        # Selection of target package
        destcls,destcxcy = cam.findDestcxcy(lconf,lcls,lcxcy)
        destcls = cam.model_intel.names[int(destcls)]

        # Step2:
        # aruco detection and calculating pixel to mm value
        acxcy,mmpp = cam.findArucoMarker(draw=True)
        if not acxcy:   # checking for detection
            continue

        # pixel to real world coordinates are found (x,y) -> real world coordinates
        dx,dy = cal_coord(destcxcy,acxcy,mmpp)
        if not dx:     # checking for coordinate within tote
            continue

        # Step3:
        ardu.moveTo(dx,dy)  # moving to target coordinate(x,y)

        # Step4:
        ardu.Zmovedown()  # z-axis movement until end-effector touches the package
        ardu.dcvON()  # turning ON Suction
        ardu.Zmoveup()  # moving to the height (4000 ticks)

        # Step5:
        # Depending on package found(i.e, front/back/side)
        # > the process varies: (there must be if-else condition to check the detected packages)

        # 1.If top(front):
        up_pkg = ["ubox","uflyer"]
        if destcls in up_pkg:
            ardu.moveToDropZone()  # move to dropzone

        # 2.If back:
        bck_pkg = ["bbox","bflyer"]
        if destcls in bck_pkg:
            ardu.moveToFlipZone()  # move to flipzone
            inFlipZone = True  # setting flag

        # 3.If side:
        side_pkg = ["sbox","sflyer"]
        if destcls in side_pkg:
            ardu.moveToFlipZone()  # move to flipzone
            inFlipZone = True  # setting flag
            # Servo control is done (2nd cam is used) (***WIP***)
            while ardu.servoValue > 0:
                cur_frame = cam.cam2_img()
                lco,lcs = cam.cam2_detect(cur_frame)
                if not lco:
                    continue
                max_conf = max(lco)
                i_conf = lco.index(max_conf)
                f_cls = lcs[i_conf]
                f_cls = cam.model_econ.names[int(f_cls)]
                if f_cls == "ubox":
                    ardu.servoValue -= 90
                    ardu.servoSetAngle(ardu.servoValue)
                    break
                elif f_cls == "bbox":
                    ardu.servoValue += 90
                    ardu.servoSetAngle(ardu.servoValue)
                    break
                else:
                    ardu.servoValue -= 10
                    ardu.servoSetAngle(ardu.servoValue)


        # Step6:
        ardu.Zmovedown_dropzone() # move to height (6000)
        ardu.dcvOFF()  # turning off suction
        ardu.Zmoveup()  # moveto home height (4000)
        ardu.servoValue = 90
        ardu.servoSetAngle(ardu.servoValue)  # changing to default servo value

        # Step7:
        if inFlipZone:
            ardu.startFlip()  # starting fliping process
            inFlipZone = False
        
        # Repeat the steps from 1 to 7 until end
        # End of loop
    

#objects:
ardu = arm_control(serialport="COM5")
cam = camera()
thread1 = threading.Thread(target=cam.cam_feed)
thread2 = threading.Thread(target=cam.view)
thread1.daemon=True
thread2.daemon=True  # detection goes off when this is commented
thread1.start()
thread2.start()
time.sleep(5)  # time taken for the yolo model to load

# Flags:
inFlipZone = False

if __name__ == '__main__':
    main()
    ardu.arduStop()


# Reference:
# 1. When arm_control() is executed: Connection is established, homing is done 
# 2. cam.cam_feed() returns the camera feed frame by frame
# 3. cam.view() -> does the imshow() and performs aruco and object detection
# 4. (2 and 3) -> are done in different threads parallelly