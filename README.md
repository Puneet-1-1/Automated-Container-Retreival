# Automated-Container-Retreival
This project developed a robust stereo vision system using open-source tools (Python, OpenCV, NumPy, Matplotlib) to achieve precise 3D reconstruction and depth mapping for robotic bin picking in unstructured environments, demonstrating high accuracy and adaptability.


# Robotic Bin Picking with Stereo Vision

## Project Overview

This project addresses the complex challenge of robotic bin picking in unstructured environments by developing and implementing a robust stereo vision-based system. [cite_start]It focuses on leveraging open-source software tools to enable precise 3D reconstruction of objects and accurate depth mapping, paving the way for enhanced automation in industrial and logistical processes. 

## Key Features & Functionality

* [cite_start]**Real-time Dual-Camera System:** Implemented a system for capturing, processing, and displaying real-time images simultaneously from two Vimba-compatible cameras, designed for interactive use with on-demand saving and software-triggered acquisition. 
* [cite_start]**Precise Stereo Calibration:** Developed a meticulous calibration process to determine the precise relationship between the two cameras (intrinsic and extrinsic parameters), crucial for accurate depth information conversion. 
* [cite_start]**Advanced Stereo Rectification:** Computes transformations to align images so corresponding points lie on the same horizontal line, simplifying disparity calculation. 
* [cite_start]**Robust Disparity & Depth Estimation:** Utilizes the Semi-Global Block Matching (SGBM) algorithm (specifically `STEREO_SGBM_MODE_SGBM_3WAY`) and Weighted Least Squares (WLS) filtering for accurate disparity maps and 3D point cloud generation. 
* [cite_start]**Interactive Disparity Map GUI:** Features a graphical user interface (GUI) built with Tkinter for real-time, interactive exploration of disparity maps, allowing users to adjust SGBM parameters and visualize their impact on depth estimation.  [cite_start]Users can click on any point to view its precise disparity value. 

## Technical Details & Methodology

* [cite_start]**Software Tools:** Developed primarily using Python 3.11  and leverages key open-source libraries:
    * [cite_start]`OpenCV` (cv2): Comprehensive library for computer vision and image processing, used for feature detection, image manipulation, and disparity calculations. 
    * [cite_start]`NumPy`: Fundamental library for efficient numerical operations on image data. 
    * [cite_start]`Matplotlib`: Used for data visualization, including displaying disparity maps. 
    * [cite_start]`vmbpy` (Vimba Python SDK): Essential for communication and control of Allied Vision machine vision cameras for image acquisition. 
    * [cite_start]`Tkinter`: For building the interactive GUI for disparity map exploration. 
    * [cite_start]`PIL` (Python Imaging Library): For image loading and manipulation within the GUI. 
* **Calibration Process:**
    * [cite_start]Individual camera calibration to find intrinsic parameters (focal length, principal point, distortion coefficients). 
    * [cite_start]Stereo calibration to determine extrinsic parameters (rotation matrix, translation vector between cameras). 
    * [cite_start]Saved calibration parameters (e.g., camera matrices, distortion coefficients, rectification maps) for reusability. 
* **Disparity Estimation:**
    * [cite_start]Employs `cv2.StereoSGBM_create` with adjustable parameters (e.g., `minDisparity`, `numDisparities`, `blockSize`, `P1`, `P2`, `uniquenessRatio`, `speckleWindowSize`, `speckleRange`). 
    * [cite_start]Incorporates `cv2.ximgproc.createRightMatcher` and a Weighted Least Squares (WLS) filter to improve disparity map quality and preserve object boundaries. 
    * [cite_start]Reprojection of disparity maps to 3D points using the `Q` matrix to obtain depth information. 

## Results & Impact

* [cite_start]**High Accuracy:** Achieved 98% accuracy in 3D object reconstruction in unstructured environments. 
* [cite_start]**Reduced Occlusion Errors:** Object occlusion errors were reduced by 30% through a novel algorithm and precise stereo camera calibration. 
* [cite_start]**Enhanced Processing Speed:** Implemented a real-time dual-camera system with software triggering, enhancing image capture and processing speed by 25%. 
* [cite_start]**Improved Depth Estimation:** GUI for real-time visualization of disparity maps and depth analysis enabled parameter optimization and a 30% reduction in depth estimation errors. 
* [cite_start]**Demonstrated Robustness:** System accuracy and robustness were demonstrated through extensive testing in simulated scenarios, achieving a 95% success rate in object tasks. 
* [cite_start]**Future Automation:** This project constitutes a significant advancement in robotic bin picking, paving the way for wider adoption of automation in manufacturing, logistics, and other industries. 

## Getting Started (for review/reproduction)

* **Dependencies:**
    * Python 3.x
    * [cite_start]`vmbpy` (Vimba Python SDK - install from Allied Vision website) 
    * [cite_start]`opencv-python` (`pip install opencv-python`) 
    * [cite_start]`numpy` (`pip install numpy`) 
    * [cite_start]`matplotlib` (`pip install matplotlib`) 
    * [cite_start]`Pillow` (`pip install Pillow`) (for Tkinter image handling) 
    * [cite_start]`tkinter` (usually built-in with Python, if not, install relevant `python3-tk` package for Linux) 
* **Calibration:**
    1.  [cite_start]Ensure Allied Vision cameras are properly connected and detected by Vimba SDK. 
    2.  Run the `StereoCalibration.py` script to generate and save calibration parameters.
* **Running the GUI:**
    1.  Ensure you have `ImageL_1.bmp` and `ImageR_1.bmp` (example stereo images) in the same directory as `DisparityGUI.py`.
    2.  Run `DisparityGUI.py` to launch the interactive disparity map explorer.

## Author

Puneet Singh Arora
[LinkedIn Profile](https://www.linkedin.com/in/puneet-singh11/)
