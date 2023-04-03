import numpy as np
import cv2

# Calibrate Camera Seperately (Use Intrinsic.py)
objp_l = np.load('objp_l.npy')
mtx_l = np.load('mtx_l.npy')
dist_l = np.load('dist_l.npy')
newcameramtx_l = np.load('newcameramtx_l.npy')

objp_r = np.load('objp_r.npy')
mtx_r = np.load('mtx_r.npy')
dist_r = np.load('dist_r.npy')
newcameramtx_r = np.load('newcameramtx_r.npy')

capture_l = cv2.VideoCapture(0)
capture_l.open(4)

capture_r = cv2.VideoCapture(1)
capture_r.open(6)

while True:
    ret_l, frame_l = capture_l.read()
    frame_l = cv2.resize(frame_l, (640, 360), interpolation=cv2.INTER_CUBIC)
    frame_l = frame_l[25:, :]
    # frame_l = cv2.undistort(frame_l, mtx_l, dist_l, None, newcameramtx_l)
    ret_cb_l, corners_l = cv2.findChessboardCorners(frame_l, (8,5), None)

    ret_r, frame_r = capture_r.read()
    frame_r = cv2.resize(frame_r, (640, 360), interpolation=cv2.INTER_CUBIC)
    frame_r = frame_r[25:, :]
    # frame_r = cv2.undistort(frame_r, mtx_r, dist_r, None, newcameramtx_r)
    ret_cb_r, corners_r = cv2.findChessboardCorners(frame_r, (8,5), None)

    cv2.imshow('frame_l', frame_l)
    cv2.imshow('frame_r', frame_r)
    print(ret_cb_l, ret_cb_r, 'find or not')
    if ret_cb_l == True and ret_cb_r == True:
        rtval_l, rvec_l, tvec_l = cv2.solvePnP(objp_l, corners_l, newcameramtx_l, dist_l)
        R3x3_l, Jacobian_l  = cv2.Rodrigues(rvec_l)

        rtval_r, rvec_r, tvec_r = cv2.solvePnP(objp_r, corners_r, newcameramtx_r, dist_r)
        R3x3_r, Jacobian_r  = cv2.Rodrigues(rvec_r)

        R3x3_l2r = R3x3_r.dot(R3x3_l.T)
        tvec_l2r = R3x3_r.dot(-R3x3_l.T.dot(tvec_l)) + tvec_r
        M3x4_l2r = np.concatenate((R3x3_l2r, tvec_l2r), axis=1)

        np.save('R3x3_l', R3x3_l)
        np.save('tvec_l', tvec_l)

        np.save('R3x3_r', R3x3_r)
        np.save('tvec_r', tvec_r)

        np.save('R3x3_l2r', R3x3_l2r)
        np.save('tvec_l2r', tvec_l2r)
        np.save('M3x4_l2r', M3x4_l2r)
        if rtval_l == True and rtval_r == True:
            break
    if cv2.waitKey(100) & 0xff == ord('q') or ret_l != True or ret_r != True: # press q to quit
        break