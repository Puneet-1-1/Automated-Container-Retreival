import cv2 as cv
import numpy as np
import pickle
import Calibration


def activateTrackbars(x):
    
    pass
def createTrackbars():
    cv.namedWindow('disparity',cv.WINDOW_NORMAL)
    cv.resizeWindow('disparity',600,600)
    
    cv.createTrackbar('minDisparities', 'disparity', -128,80, activateTrackbars)
    cv.createTrackbar('numDisparities', 'disparity', 2, 10, activateTrackbars)
    cv.createTrackbar('blockSize', 'disparity',1,10, activateTrackbars)
    cv.createTrackbar('P1_smoothness', 'disparity', 1,10, activateTrackbars)
    cv.createTrackbar('P2_smoothness', 'disparity', 1,10, activateTrackbars)
    cv.createTrackbar('disp12MaxDiff', 'disparity', 2,25, activateTrackbars)
    cv.createTrackbar('preFilterCap', 'disparity', 2,21, activateTrackbars)
    cv.createTrackbar('uniquenessRatio', 'disparity', 5,21, activateTrackbars)
    cv.createTrackbar('speckleWindowSize', 'disparity', 50,200, activateTrackbars)
    cv.createTrackbar('speckleRange', 'disparity', 1,7, activateTrackbars)
    cv.createTrackbar('saveParameters', 'disparity',0,1, activateTrackbars)

    
def disaprityCalculate(imageL, imageR):
    stereo = cv.StereoSGBM.create()

    minDisparities = cv.getTrackbarPos('minDisparities', 'disparity')
    numDisparities = cv.getTrackbarPos('numDisparities', 'disparity') * 16
    if numDisparities == 0:
        numDisparities = 16
    blockSize = cv.getTrackbarPos('blockSize', 'disparity')*2 +1
    if blockSize == 0:
        blockSize = 1
    P1_smoothness = cv.getTrackbarPos('P1_smoothness', 'disparity')*8 * 1* blockSize*blockSize
    P2_smoothness = cv.getTrackbarPos('P2_smoothness', 'disparity')*8 * 1* blockSize*blockSize
    disp12MaxDiff = cv.getTrackbarPos('disp12MaxDiff', 'disparity')
    preFilterCap = cv.getTrackbarPos('preFilterCap', 'disparity')
    if preFilterCap == 0:
        preFilterCap = 1
    uniquenessRatio = cv.getTrackbarPos('uniquenessRatio', 'disparity')
    speckleWindowSize = cv.getTrackbarPos('speckleWindowSize', 'disparity')
    speckleRange = cv.getTrackbarPos('speckleRange', 'disparity') * 16
    if speckleRange == 0:
        speckleRange =1
    
    saveParams = cv.getTrackbarPos('saveParameters', 'disparity')
    if saveParams == 1:
        
        with open('parameters.txt','w') as file:
            file.write(f"minDisparities: {stereo.getMinDisparity()}\n")
        
            file.write(f"numDisparities: {stereo.getNumDisparities()}\n")
            file.write(f"blockSize: {stereo.getBlockSize()}")
            file.write(f"uniquenessRatio: {stereo.getUniquenessRatio()}\n")
            file.write(f"speckleWindowSize: {stereo.getSpeckleWindowSize()}\n")
            file.write(f"speckleRange: {stereo.getSpeckleRange()}\n")
            print('Parameters Saved!!!')
            cv.setTrackbarPos('saveParameters','disparity', 0)

    stereo.setMinDisparity(minDisparities)
    stereo.setNumDisparities(numDisparities)
    stereo.setBlockSize(blockSize)
    stereo.setP1(P1_smoothness)
    stereo.setP2(P2_smoothness)
    stereo.setDisp12MaxDiff(disp12MaxDiff)
    stereo.setPreFilterCap(preFilterCap)
    stereo.setUniquenessRatio(uniquenessRatio)
    stereo.setSpeckleWindowSize(speckleWindowSize)
    stereo.setSpeckleRange(speckleRange)
    
    disparity = stereo.compute(imageL, imageR)
    
     # Converting to float32 
    disparityCon32 = disparity.astype(np.float32)

    # Scaling down the disparity values and normalizing them 
    disparityN = (disparityCon32/16.0 - minDisparities)/numDisparities
    
    disparityNormalized = cv.normalize(disparityN, disparityN, 0,255, cv.NORM_MINMAX)
    
    disp_color = cv.applyColorMap(disparityNormalized.astype(np.uint8), cv.COLORMAP_JET)

    return disp_color
    

def disparity(imageL, imageR):
    
    createTrackbars()       
    
    stereo = cv.StereoSGBM.create()
    
    while True:
        
        minDisparities = cv.getTrackbarPos('minDisparities', 'disparity')
        numDisparities = cv.getTrackbarPos('numDisparities', 'disparity') * 16
        if numDisparities == 0:
            numDisparities = 16
        blockSize = cv.getTrackbarPos('blockSize', 'disparity')*2 +1
        if blockSize == 0:
            blockSize = 1
        P1_smoothness = cv.getTrackbarPos('P1_smoothness', 'disparity')*8 * 1* blockSize*blockSize
        P2_smoothness = cv.getTrackbarPos('P2_smoothness', 'disparity')*8 * 1* blockSize*blockSize
        disp12MaxDiff = cv.getTrackbarPos('disp12MaxDiff', 'disparity')
        preFilterCap = cv.getTrackbarPos('preFilterCap', 'disparity')
        if preFilterCap == 0:
            preFilterCap = 1
        uniquenessRatio = cv.getTrackbarPos('uniquenessRatio', 'disparity')
        speckleWindowSize = cv.getTrackbarPos('speckleWindowSize', 'disparity')
        speckleRange = cv.getTrackbarPos('speckleRange', 'disparity') * 16
        if speckleRange == 0:
            speckleRange =1
        
        saveParams = cv.getTrackbarPos('saveParameters', 'disparity')
        
        if saveParams == 1:
            with open('parameters.txt','w') as file:
                file.write(f"minDisparities: {stereo.getMinDisparity()}\n")
            
                file.write(f"numDisparities: {stereo.getNumDisparities()}\n")
                file.write(f"blockSize: {stereo.getBlockSize()}")
                file.write(f"uniquenessRatio: {stereo.getUniquenessRatio()}\n")
                file.write(f"speckleWindowSize: {stereo.getSpeckleWindowSize()}\n")
                file.write(f"speckleRange: {stereo.getSpeckleRange()}\n")
                print('Parameters Saved!!!')
                cv.setTrackbarPos('saveParameters','disparity', 0)
                
            
            
        stereo.setMinDisparity(minDisparities)
        stereo.setNumDisparities(numDisparities)
        stereo.setBlockSize(blockSize)
        stereo.setP1(P1_smoothness)
        stereo.setP2(P2_smoothness)
        stereo.setDisp12MaxDiff(disp12MaxDiff)
        stereo.setPreFilterCap(preFilterCap)
        stereo.setUniquenessRatio(uniquenessRatio)
        stereo.setSpeckleWindowSize(speckleWindowSize)
        stereo.setSpeckleRange(speckleRange)
        
        disparity = stereo.compute(imageL, imageR)
        
         # Converting to float32 
        disparityCon32 = disparity.astype(np.float32)

        # Scaling down the disparity values and normalizing them 
        disparityN = (disparityCon32/16.0 - minDisparities)/numDisparities
        
        disparityNormalized = cv.normalize(disparityN, disparityN, 0,255, cv.NORM_MINMAX)
        
        disp_color = cv.applyColorMap(disparityNormalized.astype(np.uint8), cv.COLORMAP_JET)
        
      
        cv.imshow("disparityMap", disp_color)
        cv.imshow("Images", np.hstack((imageL, imageR)))
        
        baseline = 150
        focalLength = 294

        
        
        if cv.waitKey(1) & 0xFF == ord("d"):
            u,v = userPixelVales(disparity)
            depth = calculateDepth(u,v,disparity, baseline, focalLength)
            print(depth)
            continue
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv.destroyAllWindows()
    
    return disparity


def calculateDepth(u,v,disparity, baseline, focalLength):
    disparity[disparity == 0] = 1
    depth = (baseline * focalLength)/ disparity[u,v]
    return depth
        


def userPixelVales(image):
    L,W = image.shape
    # length is u, width is v
    
    pixel = input('Enter the pixel value u,v (range:0,0 to 479,639) separated by comma:')
    u,v = (pixel.split(','))
    u,v = int(u), int(v)
    if (u>=0 and v >=0 and u <=L and v <= W):
        return u,v
    else:
        u = 'please enter correct values'
        v= ' '
        return u,v
                     
##                
imageL = cv.imread("C:/Users/localuser/Documents/test/smoothImageL1.bmp")
imageR = cv.imread("C:/Users/localuser/Documents/test/smoothImageR1.bmp")

##imageL = cv.imread("C:/Users/localuser/Documents/test/im0.png")
##imageR = cv.imread("C:/Users/localuser/Documents/test/im1.png")
##imageL = cv.resize(imageL, (640,480))
##imageR = cv.resize(imageR, (640,480))
imageL  = cv.cvtColor(imageL, cv.COLOR_BGR2GRAY)
imageR  = cv.cvtColor(imageR, cv.COLOR_BGR2GRAY)

imageL, imageR = Calibration.undistort(imageL, imageR)
disparity = disparity(imageL, imageR)  
##### print(disparity.shape)   

        
        
        
        
    
    
    
    
    
    
    
    
