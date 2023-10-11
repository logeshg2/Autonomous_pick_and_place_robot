from ardu_serial import *
from obj_yolo import *

# Steps:
# 1. Run main.py and turn ON the robot 
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