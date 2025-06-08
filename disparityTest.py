import cv2 as cv
import numpy as np
from vmbpy import *
from queue import Queue
from typing import Optional
from typing import Callable
import matplotlib.pyplot as plt
import Calibration
import disparity



opencv_display_format = PixelFormat.Bgr8
def get_camera():
        
        with VmbSystem.get_instance() as vmb:
                cameras = vmb.get_all_cameras()
                if  not cameras:
                        print("Ohh Sorry!!!. No cameras Detected")
                        exit()

        return cameras

def setup_camera(cam: Camera):
        
        with cam:

                try:
                        cam.ExposureAuto.set('Continuous')
                except(AttributeError, VmbFeatureError):
                        pass

                try:
                        cam.BalanceWhiteAuto('Continuous')
                except(AttributeError, VmbFeatureError):
                        pass
                try:
                        stream = cam.get_stream()[0]
                        stream.GVSPAdjustPacketSize.run()
                        while not stream.GVSPAdjustPacketSize.is_done():
                                pass
                except(AttributeError, VmbFeatureError):
                        pass
        
class Handler:
        def __init__(self):
                self.display_queue = Queue(10)

        def get_image(self):
                return self.display_queue.get(True)

        def __call__(self, cam:Camera, stream:Stream, frame:Frame):

                if frame.get_status() == FrameStatus.Complete:
                        print('{} acquired {}'.format(cam, frame), flush = True)

                        if frame.get_pixel_format() == opencv_display_format:
                                display = frame
                        else:

                                display = frame.convert_pixel_format(opencv_display_format)
                        self.display_queue.put(display.as_opencv_image(), True)

                cam.queue_frame(frame)



loading = False


def onMouse(event, x,y, flag, disparityNormalized):
    if event == cv.EVENT_LBUTTONDOWN:
        distance = disparityNormalized[y][x]
        print("Distance in Centeimeters{}".format(distance))

        return distance

    
def stereoDepthMap(imageL, imageR, variableMapping):


    stereo = cv.StereoBM_create(numDisparities = 32, blockSize =variableMapping['SADWindowSize'])

    stereo.setPreFilterType(1)
    stereo.setPreFilterSize(variableMapping['preFilterSize'])
    stereo.setPreFilterCap(variableMapping['preFilterCap'])
    stereo.setMinDisparity(variableMapping['minDisparity'])
    stereo.setNumDisparities(variableMapping['numberDisparities'])
    stereo.setTextureThreshold(variableMapping['textureThreshold'])
    stereo.setUniquenessRatio(variableMapping['uniquenessRatio'])
    stereo.setSpeckleRange(variableMapping['speckleRange'])
    stereo.setSpeckleWindowSize(variableMapping['speckleWindowSize'])

    disparity = stereo.compute(imageL, imageR)
    disparityNormalized = cv.normalize(disparity, None, 0,255, cv.NORM_MINMAX)

    image = np.array(disparityNormalized, dtype = np.uint8)
    disparityColor = cv.applyColorMap(image, cv.COLORMAP_JET)

    return disparityColor, disparityNormalized

    
def activateTrackbars(x):
    global loading
    loading = False


def create_trackbars() :
    global loading

    #SWS cannot be larger than the image width and image heights.
    #In this case, width = 320 and height = 240
    cv.createTrackbar("SADWindowSize", "Stereo", 115, 230, activateTrackbars)
    cv.createTrackbar("speckleWindowSize", "Stereo", 0, 300, activateTrackbars)
    cv.createTrackbar("speckleRange", "Stereo", 0, 40, activateTrackbars)
    cv.createTrackbar("uniquenessRatio", "Stereo", 1, 20, activateTrackbars)
    cv.createTrackbar("textureThreshold", "Stereo", 0, 1000, activateTrackbars)
    cv.createTrackbar("numberDisparities", "Stereo", 1, 16, activateTrackbars)
    cv.createTrackbar("minDisparity", "Stereo", -100, 200, activateTrackbars)
    cv.createTrackbar("preFilterCap", "Stereo", 1, 63, activateTrackbars)
    cv.createTrackbar("preFilterSize", "Stereo", 5, 255, activateTrackbars)
    cv.createTrackbar("Save Settings", "Stereo", 0, 1, activateTrackbars)
    cv.createTrackbar("Load Settings","Stereo", 0, 1, activateTrackbars)




def save_load_map_settings(current_save, current_load, variable_mapping):
    global loading
    if current_save != 0:
        print('Saving to file...')

        result = json.dumps({'SADWindowSize':variable_mapping["SADWindowSize"], 'preFilterSize':variable_mapping['preFilterSize'], 'preFilterCap':variable_mapping['preFilterCap'], 
                'minDisparity':variable_mapping['minDisparity'], 'numberDisparities': variable_mapping['numberDisparities'], 'textureThreshold':variable_mapping['textureThreshold'], 
                'uniquenessRatio':variable_mapping['uniquenessRatio'], 'speckleRange':variable_mapping['speckleRange'], 'speckleWindowSize':variable_mapping['speckleWindowSize']},
                sort_keys=True, indent=4, separators=(',',':'))
        fName = '../3dmap_set.txt'
        f = open (str(fName), 'w')
        f.write(result)
        f.close()
        print ('Settings saved to file '+fName)


    if current_load != 0:
        if os.path.isfile('../3dmap_set.txt') == True:
            loading = True
            fName = '../3dmap_set.txt'
            print('Loading parameters from file...')
            f=open(fName, 'r')
            data = json.load(f)

            cv.setTrackbarPos("SADWindowSize", "Stereo", data['SADWindowSize'])
            cv.setTrackbarPos("preFilterSize", "Stereo", data['preFilterSize'])
            cv.setTrackbarPos("preFilterCap", "Stereo", data['preFilterCap'])
            cv.setTrackbarPos("minDisparity", "Stereo", data['minDisparity']+100)
            cv.setTrackbarPos("numberDisparities", "Stereo", int(data['numberDisparities']/16))
            cv.setTrackbarPos("textureThreshold", "Stereo", data['textureThreshold'])
            cv.setTrackbarPos("uniquenessRatio", "Stereo", data['uniquenessRatio'])
            cv.setTrackbarPos("speckleRange", "Stereo", data['speckleRange'])
            cv.setTrackbarPos("speckleWindowSize", "Stereo", data['speckleWindowSize'])

            f.close()
            print ('Parameters loaded from file '+fName)
            print ('Redrawing depth map with loaded parameters...')
            print ('Done!') 

        else: 
            print ("File to load from doesn't exist.")



with VmbSystem.get_instance() as vmb:
        
        ## get access to the cameras
        with get_camera()[0] as camL, get_camera()[1] as camR:
                ##print(camR)

                ## setup cameras parameters
                setup_camera(camR)
                setup_camera(camL)

                handlerR = Handler()
                handlerL = Handler()
                
                ## video start streaming
                try:
                        camR.start_streaming(handler = handlerR, buffer_count = 10)
                        camL.start_streaming(handler = handlerL, buffer_count = 10)

                        msg = 'Stream from \'{}\. Press Esc key to stop stream'
                        escKey = 27
                        counter = 0
                        size = (640, 480)
                        


                        while True:
                                
                                key = cv.waitKey(1)
                                if key == escKey:
##                                        cv.destroyWindow(msg.format(camR.get_name()))
##                                        cv.destroyWindow(msg.format(camL.get_name()))
                                        camL.stop_streaming
                                        camR.stop_streaming                                        
                                        cv.destroyAllWindows()
                                        break
##                                else if key == ord(s):
##                                        cv.imwrite('Disparity.bmp',disparitymap)

                                
                               ## get the image
                                displayR = handlerR.get_image()
                                displayL = handlerL.get_image()

                                ##resize the image to 640x480
                                imageR = cv.resize(displayR, size)
                                imageL = cv.resize(displayL, size)

                                ## display the image
##                                cv.imshow(msg.format(camR.get_name()), imageL)
##                                cv.imshow(msg.format(camL.get_name()), imageR)
##                                                               
                                ## convert BGR to Grayscale Image
                                grayL = cv.cvtColor(imageL, cv.COLOR_BGR2GRAY)
                                grayR = cv.cvtColor(imageR, cv.COLOR_BGR2GRAY)
                                
##                                cv.imshow('CamL' + msg.format(camL.get_name()), grayL)
##                                cv.imshow('CamR'+ msg.format(camR.get_name()), grayR)
                                

##                                ## undistort the image
                                
##                                undistortL, undistortR = Calibration.undistort(grayL, grayR)
##                                cv.imshow('Undistorted Left Camera', undistortL)
##                                cv.imshow('Undistorted Right Camera', undistortR)

                                channe1 = 1

##                                ## undistort using Rectifymap function
                                #grayL[:3] =1
                                #grayR[:3] =1
                                
                                #grayL = grayL.reshape((480,640,1))
                                #grayR = grayR.reshape((480,640,1))
                                print(grayL.shape, grayR.shape)
                                undistortmapL, undistortmapR = Calibration.undistortRectify(grayL, grayR)
                                
                                cv.imshow("UndistortL", undistortmapL)
                                cv.imshow("UndistortR", undistortmapR)
                                cv.namedWindow("Stereo")
                                create_trackbars()

                                variables = ["SADWindowSize", "speckleWindowSize", "speckleRange", "uniquenessRatio", "textureThreshold",
                                             "numberDisparities","minDisparity", "preFilterCap", "preFilterSize"]
                                variable_mapping = {"SADWindowSize" : 15, "speckleWindowSize" : 100, "speckleRange" : 15, "uniquenessRatio" : 10,
                                                    "textureThreshold" : 100, "numberDisparities" : 16,
                                                    "minDisparity": -25, "preFilterCap" : 30, "preFilterSize" : 105}

                                 #getting trackbar position and assigning to the variables
                                
                                if loading == False:
                                        for v in variables:
                                                current_value = cv.getTrackbarPos(v, "Stereo")
                                                if v == "SWS" or v == "PreFiltSize":
                                                        if current_value < 5:
                                                                
                                                                current_value = 5
                                                        elif current_value % 2 == 0:
                                                                current_value += 1
                                                if v == "NumofDisp":
                                                         if current_value == 0:
                                                                 current_value = 1
                                                                 current_value = current_value * 16
                                                if v == "MinDisp":
                                                        current_value = current_value - 100
                                                if v == "UniqRatio" or v == "PreFiltCap":
                                                        if current_value == 0:
                                                                current_value = 1
                                                
                                                variable_mapping[v] = current_value

                            #getting save and load trackbar positions
                        

                                current_save = cv.getTrackbarPos("Save Settings", "Stereo")
                                current_load = cv.getTrackbarPos("Load Settings", "Stereo")
                                save_load_map_settings(current_save, current_load, variable_mapping)
                                cv.setTrackbarPos("Save Settings", "Stereo", 0)
                                cv.setTrackbarPos("Load Settings", "Stereo", 0)
                                disparity_color, disparity_normalized = stereoDepthMap(undistortmapL, undistortmapR, variable_mapping)

                                cv.imshow("Image",disparity_color)

















                                
####                                
##                                disparityColor, disparityNormalized = disparity.stereoDepthMap(undistortmapL, undistortmapR)
##
####                                cv.setMouseCallback("Depthmap", onMouse, disparityNormalized)
##
##                                #Show depth map and image frames
##                                output = cv.addWeighted(imageL, 0.5, disparityColor, 0.5, 0.5)
##                                cv.imshow("Depthmap", np.hstack((disparityColor, output)))
##                                
####                                cv.imshow('Disparity', disparityColor)
######                                
##
                                
                finally:
                        camL.stop_streaming()
                        camR.stop_streaming()


