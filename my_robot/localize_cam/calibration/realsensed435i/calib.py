import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt 

# Defining the dimensions of checkerboard
CHECKERBOARD_SIZE = (6,8)
REAL_SIZE = 6
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Creating vector to store 3D points
objpoints = []
# Creating vector to store 2D points
imgpoints = []

# Defining world coords for 3D points
objp = np.zeros((1, CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD_SIZE[0],0:CHECKERBOARD_SIZE[1]].T.reshape(-1, 2) * REAL_SIZE
prev_img_shape = None

# Extracting path of individual image stored in a given directory
images = glob.glob('/home/trungle/catkin_ws/src/calibration/realsensed435i/calib_images/*.png')
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Find the chess board corners
    # If desired number of corners are found in the image
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_SIZE, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    """
    If desired number of corner are detected, we refine the pixel coordinates and
    display them on the images of checker board
    """
    if ret == True:
        objpoints.append(objp)
        # refining pixel coordinates for given 2d points
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # draw and display the corners
        img = cv2.drawChessboardCorners(img, CHECKERBOARD_SIZE, corners2, ret)

    cv2.imshow('img', img)
    cv2.waitKey(0)

h,w = img.shape[:2]
"""
Performing camera calibration by 
passing the value of known 3D points (objpoints)
and corresponding pixel coordinates of the 
detected corners (imgpoints)
"""
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera matrix : \n")
print(mtx)
print("dist : \n")
print(dist)
print("rvecs : \n")
print(rvecs)
print("tvecs : \n")
print(tvecs)

# save calibration parameters
np.savez('/home/trungle/catkin_ws/src/calibration/realsensed435i/calib_savez', cameraMatrix=mtx, distCoef=dist, rVectors=rvecs, tVectors=tvecs)