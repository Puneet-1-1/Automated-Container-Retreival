import cv2 as cv
import numpy as np
import glob
import pickle

boardsize = (7,9)
framesize = (640,480)

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objP = np.zeros((boardsize[0]*boardsize[1],3), np.float32)
objP[:,:2] = np.mgrid[0:boardsize[0],0:boardsize[1]].T.reshape(-1,2)
 
# Arrays to store object points and image points from all the images.
objPoints = [] # 3d point in real world space
imgPoints = [] # 2d points in image plane.
 
# images = glob.glob('Images\cam0Img\*.bmp')
images = glob.glob("C:/Users/localuser/Documents/test/CamR/*.bmp")
##images = glob.glob("C:/Users/localuser/Documents/test/CamL/*.bmp")
 
for image in images:
    img = cv.imread(image)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, boardsize, None)
    
    # If found, add object points, image points (after refining them)
    if ret == True:
                
        objPoints.append(objP)
        
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgPoints.append(corners2)
        
        # Draw and display the corners
        cv.drawChessboardCorners(img, boardsize, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(1000)
    
cv.destroyAllWindows()

############### Get Intrinscic and Extrinsic Parametes by CALIBRATION ########################
calibration, cameraMatrix, distotion, rotVector, transVector = cv.calibrateCamera(objPoints, imgPoints, framesize, None, None)

print("Camera Calibrated \n", calibration)
print("Camera matrix: \n", cameraMatrix)
print("Distortions =  \n", distotion)
print("Roatation Vectot = \n", rotVector)
print("Translation Vector \n", transVector)

# pickle.dump((cameraMatrix, distotion), open("calibration.pkl", "wb"))
# pickle.dump((cameraMatrix, open("cameraMatrix.pkl", "wb")))
# pickle.dump((distotion, open("distCoeffs.pkl", "wb")))
# rotV = [1,2,3]
# from scipy.spatial.transform import Rotation
# rotation_obj = Rotation.from_rotvec(rotV)
# rotation_matrix = rotation_obj.as_matrix

rotation_matrix,s= cv.Rodrigues(rotVector[0])

print("Rotation Matrix: ", rotation_matrix)


################# UNDISTORTION ###########################
# img = cv.imread('Images\cam1Img\image1_0.png')
img = cv.imread('C:/Users/localuser/Documents/test/pixel2WCos/TestImages/Cam1/Image1_6.bmp')
height, width = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, distotion, (width,height), 1, (width,height))


########## undistort with undistort function
dst = cv.undistort(img, cameraMatrix, distotion, None, newcameramtx)
 
# crop the image
x, y, width, height = roi
dst = dst[y:y+height, x:x+width]
# cv.imwrite('calibresult.png', dst)

##### undistort with remapping
mapx, mapy = cv.initUndistortRectifyMap(cameraMatrix, distotion, None, newcameramtx, (width,height), 5)
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
 
# crop the image
x, y, width, height = roi
dst = dst[y:y+height, x:x+width]
##cv.imwrite('calibresult1.bmp', dst)

########### REPROJECTION ERROR ###########

mean_error = 0
for i in range(len(objPoints)):
 imgpoints2, _ = cv.projectPoints(objPoints[i], rotVector[i], transVector[i], cameraMatrix, distotion)
 error = cv.norm(imgPoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
 mean_error += error
 
print( "\n total error: {}".format(mean_error/len(objPoints)) )

