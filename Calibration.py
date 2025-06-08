import sys
import cv2 as cv
import numpy as np

## Load the Parameters
camMatrix0 = np.load('C:/Users/localuser/Documents/test/pixel2WCos/calibParams/cameraMatrixLeft.npy')
camMatrix1 = np.load('C:/Users/localuser/Documents/test/pixel2WCos/calibParams/cameraMatrixRight.npy')
distcoef0 = np.load('C:/Users/localuser/Documents/test/pixel2WCos/calibParams/Distortion_CoefficentLeft.npy')
distcoef1 = np.load('C:/Users/localuser/Documents/test/pixel2WCos/calibParams/Distortion_CoefficentRight.npy')
newCamMat0 = np.load('C:/Users/localuser/Documents/test/pixel2WCos/calibParams/newcameraMatrixLeft.npy')
newCamMat1 = np.load('C:/Users/localuser/Documents/test/pixel2WCos/calibParams/newcameraMatrixRight.npy')
rectTransL = np.load('C:/Users/localuser/Documents/test/pixel2WCos/calibParams/rotationMatrixLeft.npy' )
rectTransR = np.load('C:/Users/localuser/Documents/test/pixel2WCos/calibParams/rotationMatrixRight.npy')
projMatrixL = np.load('C:/Users/localuser/Documents/test/pixel2WCos/calibParams/projectionMatrixLeft.npy' )
projMatrixR = np.load('C:/Users/localuser/Documents/test/pixel2WCos/calibParams/projectionMatrixRight.npy')
    

def undistort(image0, image1):
    
    
    #The function is simply a combination of initUndistortRectifyMap (with unity R ) and remap (with bilinear interpolation)
    undistorted0 = cv.undistort(image0, camMatrix0, distcoef0, newCamMat0)
    undistorted1 = cv.undistort(image1, camMatrix1, distcoef1, newCamMat1)
    
    return undistorted0, undistorted1


def undistortRectify(imageL, imageR):

    ## Load the parameteres
    cv_file = cv.FileStorage()
##    cv_file.open('C:/Users/localuser/Documents/test/pixel2WCos/calibParams/stereoMap.xml', cv.FileStorage_READ)
##    stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
##    stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
##    stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
##    stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()
    
    
    stereoMapL_x, stereoMapL_y = cv.initUndistortRectifyMap(camMatrix0, distcoef0,rectTransL, projMatrixL, (640,480), 5)

    stereoMapR_x, stereoMapR_y = cv.initUndistortRectifyMap(camMatrix1, distcoef1, rectTransR, projMatrixR, (640,480), 5)
    
    ##undistort and rectify images
    undistortedL = cv.remap(imageL, stereoMapL_x, stereoMapL_y, cv.INTER_LINEAR)
    undistortedR = cv.remap(imageR, stereoMapR_x, stereoMapR_y, cv.INTER_LINEAR)

    return undistortedL, undistortedR


