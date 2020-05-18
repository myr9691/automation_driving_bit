import numpy as np
import cv2

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
wc = 10  ## chessboard horizontal pattern count
hc = 7  ## chessboard vertical pattern count
objp = np.zeros((wc * hc, 3), np.float32)
objp[:, :2] = np.mgrid[0:wc, 0:hc].T.reshape(-1, 2)

objpoints = []
imgpoints = []

file = 'C:\\Users\\bit\\Downloads\\new test.png'  ## chessboard pattern image
dist_file = 'C:\\Users\\bit\\Downloads\\lane_test.png'  ## distortion image(same distortion with chessboard image)

img = cv2.imread(file)  ## Read chessboard image
_img = cv2.resize(img, dsize = (640, 480), interpolation = cv2.INTER_AREA)  ## If image size is too large, resize
# cv2.imshow('img', _img)
src_img = cv2.imread(dist_file)  ## Read distortion image
dist_img = cv2.resize(src_img, dsize = (640, 480), interpolation = cv2.INTER_AREA)
gray = cv2.cvtColor(_img, cv2.COLOR_BGR2GRAY)
# cv2.imshow('dist', dist_img)
# cv2.waitKey(0)

ret, corners = cv2.findChessboardCorners(gray, (wc, hc), None)
print(ret)  ## If ret is false, check wc, hc number or change image to clearer distortion image

if ret == True:
    objpoints.append(objp)

    corners2 = cv2.cornerSubPix(gray, corners, (10, 10), (-1, -1), criteria)
    imgpoints.append(corners2)

    # Draw and display the corners
    img = cv2.drawChessboardCorners(_img, (wc, hc), corners2, ret)
    # cv2.imshow('img', img)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0)  
    ## getOptimalNewCameraMatrix parameter alpha
    ## Free scaling parameter 
    ## between 0 (when all the pixels in the undistorted image are valid) 
    ## and 1 (when all the source image pixels are retained in the undistorted image)
    dst = cv2.undistort(dist_img, mtx, dist)
    dst2 = cv2.undistort(dist_img, mtx, dist, None, newcameramtx)
    cv2.imshow('num1', dst)
    cv2.imshow('num2', dst2)
    cv2.waitKey(0)

cv2.destroyAllWindows()
