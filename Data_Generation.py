import Motor_Control as MC
import SFS
import img_process as img_p

import numpy as np
import cv2

# Calibrate Cam Parameter (Use Intrinsic.py)
mtx_l = np.load('mtx_l.npy')
dist_l = np.load('dist_l.npy')
newcameramtx_l = np.load('newcameramtx_l.npy')

mtx_r = np.load('mtx_r.npy')
dist_r = np.load('dist_r.npy')
newcameramtx_r = np.load('newcameramtx_r.npy')

# Calibrate Camera Pose (Use Extrinsic.py)
R3x3_l = np.load('R3x3_l.npy')
tvec_l = np.load('tvec_l.npy')

R3x3_r = np.load('R3x3_r.npy')
tvec_r = np.load('tvec_r.npy')

R3x3_l2r = np.load('R3x3_l2r.npy')
tvec_l2r = np.load('tvec_l2r.npy')
M3x4_l2r = np.load('M3x4_l2r.npy')


tvec_l_inv = np.dot(-R3x3_l.T,tvec_l)
E_ol_inv_ = np.concatenate((R3x3_l.T, tvec_l_inv), 1)
E_ol_inv = np.concatenate((E_ol_inv_, np.array([[0,0,0,1]])), 0)

tvec_r_inv = np.dot(-R3x3_r.T,tvec_r)
E_or_inv_ = np.concatenate((R3x3_r.T, tvec_r_inv), 1)
E_or_inv = np.concatenate((E_or_inv_, np.array([[0,0,0,1]])), 0)


# Capture from Camera
capture_l = cv2.VideoCapture(0)
capture_r = cv2.VideoCapture(1)

# Check
capture_l.open(0)
capture_r.open(6)

lower_green_l = np.array([0, 20, 0])     ##[R value, G value, B value]
upper_green_l = np.array([200, 255, 240])

lower_green_r = np.array([0, 20, 0])     ##[R value, G value, B value]
upper_green_r = np.array([215, 255, 240]) 

buffercount = 0

# Define Motor rotate angle
motor_angle = 10 # abs degree val

print('in while')
while True:
    check = MC.motor_control(motor_angle)
    if check != True :
        continue
    else:

        # read image from capture
        ret_l, _ = img_p.read_img(capture_l)
        ret_r, _ = img_p.read_img(capture_r)

        # apply filter and mask
        masked_image_l = img_p.chroma_key(capture_l, lower_green_l, upper_green_l)
        masked_image_r = img_p.chroma_key(capture_r, lower_green_r, upper_green_r)
        
        # Binary 
        gray_l = cv2.cvtColor(masked_image_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(masked_image_r, cv2.COLOR_BGR2GRAY)
        _, binary_l = cv2.threshold(gray_l, 6, 255, cv2.THRESH_BINARY)
        _, binary_r = cv2.threshold(gray_r, 6, 255, cv2.THRESH_BINARY)

        if buffercount <= 25:
            buffercount += 1
        else:
            XYZ_all_l = SFS.uvtoXYZ(binary_l, E_ol_inv)
            XYZ_all_r = SFS.uvtoXYZ(binary_r, E_or_inv)
            XYZ_all_l = np.array(XYZ_all_l)
            XYZ_all_r = np.array(XYZ_all_r)
            print('uv2XYZ done', len(XYZ_all_l), 'set_l len', len(XYZ_all_r), 'set_r len', len(XYZ_all_l)*len(XYZ_all_r))

            sample_rate = 1
            XYZ_all_l_sampled = XYZ_all_l[np.random.choice(len(XYZ_all_l), int(len(XYZ_all_l)*sample_rate), replace=False)]
            XYZ_all_r_sampled = XYZ_all_r[np.random.choice(len(XYZ_all_r), int(len(XYZ_all_r)*sample_rate), replace=False)]
            print('sample done', len(XYZ_all_l_sampled), 'set_l len', len(XYZ_all_r_sampled), 'set_r len', len(XYZ_all_l_sampled)*len(XYZ_all_r_sampled))

            intersect_points = SFS.findintersection(XYZ_all_l_sampled, XYZ_all_r_sampled, tvec_l, tvec_r)
            # ex pic rate = 0.005
            sampled_points = intersect_points[np.random.choice(len(intersect_points), int(len(intersect_points)*0.05), replace=False)]
            print(np.shape(sampled_points), 'size of sampled points')
            print('intersect points obtained')

            resolution = 1
            voxels = SFS.pointcloud2voxel(sampled_points, resolution)
            print('voxels obtained')

            


