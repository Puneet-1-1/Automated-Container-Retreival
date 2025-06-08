import cv2 as cv
import numpy as np
import glob

def StereoCalibration():
    ######### FIND CHESSBOARD CORNERS #######################
    boardsize = (9,7)
    framesize = (640,480)

    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    #prepare object points with numpy
    objP = np.zeros((boardsize[0]*boardsize[1],3), np.float32)
    objP[:,:2] = np.mgrid[0:boardsize[0],0:boardsize[1]].T.reshape(-1,2)
    
    # Arrays to store object points and image points from all the images.
    objPoints = [] # 3d point in real world space
    imgPointsL = [] # 2d points in image plane of camera1.
    imgPointsR = [] # 2d points in image plane of camera0.

    imagesL = glob.glob("S:\Bin Picking Project\TEST\CamL\*.bmp")
    imagesR = glob.glob("S:\Bin Picking Project\TEST\CamR\*.bmp")


    for imageL, imageR in zip(imagesL, imagesR):
        
        imgL = cv.imread(imageL)
        imgR = cv.imread(imageR)

        grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
        grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)
        
        
        #Find corners in the chessboard
        retL, cornersL = cv.findChessboardCorners(grayL, boardsize, None)
        retR, cornersR = cv.findChessboardCorners(grayR, boardsize, None)
        
        print(retL, retR)
        # If corners are found, add object points and image points
        if retL and retR == True:
            objPoints.append(objP)
            
            #append the chessboard corner point to imagepoints
            cornersL = cv.cornerSubPix(grayL, cornersL, (11,11),(-1,-1), criteria)
            imgPointsL.append(cornersL)
            cornersR = cv.cornerSubPix(grayR, cornersR, (11,11),(-1,-1), criteria)
            imgPointsR.append(cornersR)

            
            # draw and display the corners
            cv.drawChessboardCorners(imgL, boardsize,cornersL, retL)
            cv.imshow("camera1_img", imgL)        
            cv.drawChessboardCorners(imgR, boardsize,cornersR, retR)
            cv.imshow("camera0_img", imgR)

            
            cv.waitKey(1000)
            
    cv.destroyAllWindows()
            

    ##############   CALIBRATION ##############################################
    #Camera1 Caliberation
    calibration1L, cameraMatrixL, distortionL, rotVectorL, transVectorL = cv.calibrateCamera(objPoints, imgPointsL, grayL.shape[::-1], None, None)
    heightL, widthL, channelsL = imgL.shape
    #  # #### Returns the new camera intrinsic matrix based on the free scaling parameter.
    newCameraMatrixL, roi_L = cv.getOptimalNewCameraMatrix(cameraMatrixL, distortionL,(widthL, heightL), 1, (widthL, heightL))

    # Camera2 Caliberation
    calibrationR, cameraMatrixR, distortionR, rotVectorR, transVectorR = cv.calibrateCamera(objPoints, imgPointsR, grayR.shape[::-1], None, None)
    heightR, widthR, channelsR = imgR.shape
    #  # #### Returns the new camera intrinsic matrix based on the free scaling parameter.
    newCameraMatrixR, roi_R = cv.getOptimalNewCameraMatrix(cameraMatrixR, distortionR,(widthR, heightR), 1, (widthR, heightR))
    
    ################ Stereo Vision Caliberation #######################
    flags = 0
    flags |= cv.CALIB_FIX_INTRINSIC
    # here we fix the intrinsic camera matrices so that only Rotation, tanslataion essential and fundamental matrices
    # so the intrinsic parameters are the same

    ret, newCameraMatrixL, distCoeffsL, newCameraMatrixR, distCoeffsR, rotMatrix, transVector, essMatrix, fundMatrix = cv.stereoCalibrate(objPoints,  imgPointsL, imgPointsR, newCameraMatrixL, distortionL,
                                                                                                                            newCameraMatrixR, distortionR, grayR.shape[::-1], criteria=criteria, flags=flags)



    ############# Stereo Rectification #######################################
    # #  Computes rectification transforms for each head of a calibrated stereo camera
    # # # alpha = 1
    # # #alpha=0 means that the rectified images are zoomed and shifted   
    rectifyScale = 1
    rectTransL, rectTransR, projMatrixL, projMatrixR, disp2DepthMapMatrix, roiL, roiR = cv.stereoRectify(newCameraMatrixL, distCoeffsL, newCameraMatrixR, distCoeffsR, grayR.shape[::-1], rotMatrix, transVector, rectifyScale, (0,0))


    stereoMapLx, stereoMapLy = cv.initUndistortRectifyMap(cameraMatrixL, distCoeffsL, rotMatrix, projMatrixL, grayL.shape[::-1], cv.CV_32FC1)

    stereoMapRx, stereoMapRy = cv.initUndistortRectifyMap(cameraMatrixR, distCoeffsR, rotMatrix, projMatrixR, grayR.shape[::-1], cv.CV_32FC1)

    print("Saving Caliberation Parameters")
    
    np.save('S:\Bin Picking Project\TEST\pixel2WCos\calibParams\cameraMatrixLeft.npy', cameraMatrixL)
    np.save('S:\Bin Picking Project\TEST\pixel2WCos\calibParams\cameraMatrixRight.npy', cameraMatrixR)
    np.save('S:\Bin Picking Project\TEST\pixel2WCos\calibParams\newcameraMatrixLeft.npy', newCameraMatrixL)
    np.save('S:\Bin Picking Project\TEST\pixel2WCos\calibParams\newcameraMatrixRight.npy', newCameraMatrixR)
    np.save('S:\Bin Picking Project\TEST\pixel2WCos\calibParams\Distortion_CoefficentLeft.npy', distCoeffsL)
    np.save('S:\Bin Picking Project\TEST\pixel2WCos\calibParams\Distortion_CoefficentRight.npy', distCoeffsR)
    np.save('S:\Bin Picking Project\TEST\pixel2WCos\calibParams\RotationMatrixLR.npy', rotMatrix)
    np.save('S:\Bin Picking Project\TEST\pixel2WCos\calibParams\TranslationVectorLR.npy', transVector)
    np.save('S:\Bin Picking Project\TEST\pixel2WCos\calibParams\projectionMatrixLeft.npy', projMatrixL)
    np.save('S:\Bin Picking Project\TEST\pixel2WCos\calibParams\projectionMatrixRight.npy', projMatrixR)
    np.save('S:\Bin Picking Project\TEST\pixel2WCos\calibParams\rotationMatrixLeft.npy', rectTransL)
    np.save('S:\Bin Picking Project\TEST\pixel2WCos\calibParams\rotationMatrixRight.npy', rectTransR)
    np.save('S:\Bin Picking Project\TEST\pixel2WCos\calibParams\disparity2depthMapMatrix.npy',disp2DepthMapMatrix)



    print("saving Parameters")
    cv_file = cv.FileStorage('C:/Users/localuser/Documents/test/pixel2WCos/calibParams/stereoMap.xml', cv.FILE_STORAGE_WRITE)

    cv_file.write('stereoMapL_x', stereoMapLx)
    cv_file.write('stereoMapL_y', stereoMapLy)
    cv_file.write('stereoMapR_x', stereoMapRx)
    cv_file.write('stereoMapR_y', stereoMapRy)

    cv_file.release()



StereoCalibration()
