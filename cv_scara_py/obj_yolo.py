import cv2

# Note:
    # Labels => ['bbox', 'bflyer', 'sbox', 'sflyer', 'ubox', 'uflyer']
    # Priority: 1.ubox (id-4)
    #           2.uflyer (id-5)
    #           3.sbox (id-2)
    #           4.sflyer (id-3)
    #           5.bbox (id-0)
    #           6.bflyer (id-1)
    # The priority may also depend on the depth(i.e, least distance high priority)    

def destPoint(Lcls,Lconf,Lcxcy,Ldepth): 
    minDepth = min(Ldepth)  # initially taking the least depth point
    indexForDepth = Ldepth.index(minDepth)
    mincxcy = Lcxcy[indexForDepth]
    return mincxcy, minDepth

# NOTE: trolley bbox points is (414,312) & (14,-288)
def distMovement(Acxcy,Pcxcy,mm_per_pixel):  # To calculate the final coordinate point
    x1,y1 = Acxcy[0],Acxcy[1]
    x2,y2 = Pcxcy[0],Pcxcy[1]                     

    # Difference in pixel lenght between Aruco and Point
    dx, dy = x2-x1, y2-y1

    # Pixel to mm conversion:
    dxmm = dx * mm_per_pixel
    dymm = dy * mm_per_pixel

    # Initiall Acxcy is in (414,312)
    # So destination coordinate point is:
    #destx = int(414 - dymm)
    #desty = int(312 - dxmm)

    destx = int(0 - dymm)  # measurement from the centre of the robot base
    desty = int(0 - dxmm)    
    print(destx,desty)
    if (destx in range(14,414)) and (desty in range(-288,312)):
        return destx,desty
    else:
        return None