import numpy as np
import cv2
d = 2 # d: Diameter of each pixel neighborhood.
sigCol = 5 # sigmaColor: Value of \sigma in the color space. The greater the value, the colors farther to each other will start to get mixed.
sigSpa = 1 # sigmaSpace: Value of \sigma in the coordinate space. The greater its value, the more further pixels will mix together, given that their colors lie within the sigmaColor range.



def read_img(cap):
    ret, frame = cap.read()
    # resize image to correct scalepython 
    frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_CUBIC)
    frame = frame[25:, :]
    return ret, frame

def bilateral_filter(cap):
    _, frame = read_img(cap)
    # Bilateral Filter
    frame_BF = cv2.bilateralFilter(frame, d, sigCol, sigSpa)
    return frame_BF

def chroma_key(cap, lower, upper):
    frame_BF = bilateral_filter(cap)
    # Chroma Key Mask
    frame_copy = np.copy(frame_BF)
    frame_copy = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2RGB)

    mask = cv2.inRange(frame_copy, lower, upper)

    masked_image = np.copy(frame_copy)
    masked_image[mask != 0] = [0, 0, 0]
    return masked_image

# def chroma_key(cap, lower, upper):
#     frame_BF = bilateral_filter(cap)
#     # Chroma Key Mask
#     frame_copy = np.copy(frame_BF)
#     frame_copy = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2RGB)

#     mask = cv2.inRange(frame_copy, lower, upper)

#     masked_image = np.copy(frame_copy)
#     masked_image[mask != 0] = [0, 0, 0]

#     # binary
#     grayImage = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
#     binary_image = cv2.threshold(grayImage, 10, 255, cv2.THRESH_BINARY)
#     return binary_image