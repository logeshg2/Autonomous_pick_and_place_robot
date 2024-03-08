# Autonomous_pick_and_place_robot
This repository contains the script for an autonomous scara robot which is capable of picking parcel packages from a tote and placing them on a conveyor in warehouse. The autonomous operation is fully hardcoded in python. Yolov8 deep learning object detection algorithm is used to detect parcels.

<p float="left">
  <img src="https://github.com/logeshg2/Autonomous_pick_and_place_robot/blob/main/Images/Scara_Robot.jpg"  width="40%" /> 
  <img src="https://github.com/logeshg2/Autonomous_pick_and_place_robot/blob/main/Images/Scara_Robot_2.jpg"  width="40%" />
</p>

## Installation:

1. Opencv
```shell
pip install opencv-python
```

2. Ultralytics
```shell
pip install ultralytics
```

2. Pyserial
```shell
pip install pyserial
```

3. Pyrealsense2
```shell
pip install pyrealsense2
```

4. Pytorch and Cuda (Latest)

To install [Pytorch](https://pytorch.org/) and [Cuda 11.8](https://developer.nvidia.com/cuda-11-8-0-download-archive)

## Code:

1. Arduino Code: The motor control code is available in ```\Arduino_Code\Arduino_Code.ino```. 

2. ```\scara python\ardu_control.py``` contains the script for motor contorl and inverse kinematic for the scara robot.

3. ```\scara python\detection.py``` contains the script for computer vision, object detection and Intel realsense camera pipeline.

4. ```\scara python\main.py``` contains the main loop for the autonomous operation.

5. ```\scara python\yolo_weights``` contains the yolov8 custom trained weight file.

### Object Detection:
<img src="https://github.com/logeshg2/Autonomous_pick_and_place_robot/assets/121447333/7992a0c8-d150-4ad9-b999-7d1745e12015"  width="40%" /> 

### Electronics used:
<img src="https://github.com/logeshg2/Autonomous_pick_and_place_robot/blob/main/Images/Scara_Ele.jpg"  width="40%" />

### Future works:

1. ROS implementation

2. Imitation learning implementation.
