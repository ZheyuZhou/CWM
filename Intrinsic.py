import numpy as np
import cv2

# Calibrate both camera seperately for Intrinsic Matrix

# Which camera?
###############################################################
###############################################################
###############################################################
###############################################################
###############################################################
# which_cam = '_l'

# # video capture
# capture = cv2.VideoCapture(0)
# capture.open(4)

which_cam = '_r'

# video capture
capture = cv2.VideoCapture(1)
capture.open(6)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((5*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:5].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Num of data collected
num_data = 20


while True:
    ret, frame = capture.read()
    frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_CUBIC)
    frame = frame[25:, :]
    cv2.imshow("frame", frame) # display left
    ret_cb, corners = cv2.findChessboardCorners(frame, (8,5), None)
    if ret_cb == True:
        gray_masked = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = cv2.cornerSubPix(gray_masked, corners, (15, 15), (-1, -1), criteria)

        if len(objpoints) < num_data:
            objpoints.append(objp)
            imgpoints.append(corners)
        elif ret_cb != True:
            print(ret_cb, 'not found corners yet')

    if len(objpoints) == num_data:
        # calibrate
        # print(len(objpoints[0]), objpoints[0], type(objpoints), 'objpoints 3D')
        # print(len(imgpoints[0]), imgpoints[0], type(imgpoints[0]),'imgpoints 2D')
        ret_cali, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray_masked.shape[::-1],None,None)
        if ret_cali != True:
            h,  w = frame.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

            # undistort
            undst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
            print(undst.shape[:2], 'shape after undistort')
            print(mtx, dist,newcameramtx, 'output')
            np.save('mtx'+which_cam+'.npy',mtx)
            np.save('dist'+which_cam+'.npy',dist)
            np.save('newcameramtx'+which_cam+'.npy',newcameramtx)


            mean_error = 0
            for i in range(len(objpoints)):
                imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
                mean_error += error
                print( "total error: {}".format(mean_error/len(objpoints)) )

            break
    if cv2.waitKey(100) & 0xff == ord('q') or ret != True: # press q to quit
        break
capture.release()  # release cam left
cv2.destroyAllWindows() # close windows