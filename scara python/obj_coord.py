from math import *

# links
L1 = 228  # L1 = 228mm
L2 = 312  #old -> 291 # L2 = 136.5mm L3 = 154

camToJ2base = 477
J2baseToEnd = 366

def cal_moveTo(aru,tar,mmP):  # returns the movement required in x,y to reach from the aruco point to the target point
    (ax,ay) = (aru[0],aru[1])  # aruco marker -> (200,200)
    (tx,ty) = (tar[0],tar[1])

    # pixel difference:
    (dx,dy) = (tx - ax,ty-ay)
    # calculation real world distance:
    dxmm = dx * mmP
    dymm = dy * mmP
    # distance to move:
    destx = int(200 - dymm)
    desty = int(200 - dxmm)

    if (destx in range(14,414)) and (desty in range(-288,312)):
        return destx,desty
    else:
        return None,None

def cal_coord(dcxcy,acxcy,mmpp):  # dcxcy is tuple
    tx,ty = dcxcy[0],dcxcy[1]
    ax,ay = acxcy[0],acxcy[1]

    # pixel difference:
    (dx,dy) = (tx-ax,ty-ay)

    # pixel difference to mm difference:
    dxmm = dx * mmpp
    dymm = dy * mmpp

    # destination coordinate:  # the centre of tote is at coordinate (0,y) for the robot 
    destx = int(0 - dymm)
    desty = int(0 - dxmm)

    if (destx in range(75,475)) and (desty in range(-300,300)):
        return destx,desty
    else:
        return None,None


# 200 steps per mm  # camDepth is in meters
def cal_ZMovement(depth):  # this function calculates z movement required to the object
    depth = round(depth * 1000)
    J2baseTopoint = depth - camToJ2base
    EndTOpoint = J2baseTopoint - J2baseToEnd
    return EndTOpoint   # in mm 
    