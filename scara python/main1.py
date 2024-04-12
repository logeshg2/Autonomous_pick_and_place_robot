from ardu_control import arm_control
#from detection import camera
#from obj_coord import *
#import threading
import time

# main function:
def main():
    #global inFlipZone
    #ardu.moveToDropZone() # Robot waits in dropzone
    while True:
        # random motion
        ardu.moveTo(0,400)
        time.sleep(0.5)
        ardu.moveTo(200,300)
        time.sleep(0.5)
        ardu.moveTo(200,-300)
        time.sleep(0.5)
        # repeat    
    

ardu = arm_control(serialport="/dev/ttyACM0")
time.sleep(2)

if __name__ == '__main__':
    main()
    ardu.arduStop()
