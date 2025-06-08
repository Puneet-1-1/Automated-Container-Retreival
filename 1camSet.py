import cv2 as cv
import numpy as np
from vmbpy import *
from queue import Queue
from typing import Optional
from typing import Callable
import matplotlib.pyplot as plt
import Calibration
import disparity
import disparityGUI2 as disp


def onMouse(event, x,y, flag, disparityNormalized):
    if event == cv.EVENT_LBUTTONDOWN:
        distance = disparityNormalized[y][x]
        print("Distance in Centeimeters{}".format(distance))

        return distance

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
                        disp.createTrackbars()

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
                                
                                undistortmapL, undistortmapR = Calibration.undistort(grayL, grayR)
                                
                                cv.imshow("UndistortL", undistortmapL)
                                cv.imshow("UndistortR", undistortmapR)
##                                
##                                disparityColor, disparityNormalized = disparity.stereoDepthMap(undistortmapL, undistortmapR)

                                disparity = disp.disaprityCalculate(undistortmapL, undistortmapR )
##                                disparity = disp.disaprityCalculate(grayL, grayR )
                                cv.imshow('Disparity', disparity)

##                                cv.setMouseCallback("Depthmap", onMouse, disparityNormalized)

                                #Show depth map and image frames
##                                output = cv.addWeighted(imageL, 0.5, disparityColor, 0.5, 0.5)
##                                cv.imshow("Depthmap", np.hstack((disparityColor, output)))
##                                
##                                cv.imshow('Disparity', disparityColor)
####                                
##
                                
                finally:
                        camL.stop_streaming()
                        camR.stop_streaming()


