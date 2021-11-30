#!/usr/bin/env python

import cv2
import numpy as np
import math

# ========================= Student's code starts here =========================

# Params for camera calibration
O_r = 240 
O_c = 320
beta = (290 - 266)/.0318 #pixels/meter 750

def IMG2W(x,y):
    # print(x,y)
    x_c = (y-O_r+212)/beta
    y_c = (x-O_c+78)/beta
    z_c = .035

    p = np.array([[x_c],[y_c],[z_c]])

    return p

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 120
    params.maxArea = 700

    # Filter by Circularity
    params.filterByCircularity = False

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    lower = (6,150,150)     # orange lower
    upper = (14,255,255)   # orange upper

    if (color == 'green'):
        lower = (0,100,100)     # red lower 
        upper = (24,255,255)   # red upper 

    if (color == 'purple'):
        lower = (25,100,200)     # yellow lower
        upper = (35,255,255)   # yellow upper

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))


    # print(blob_image_center)

    # blob_c = np.zeros((10,2))
    # for i in range(10):
    #     blob_c.append(detector.detect(mask_image))
    # math.atan2(blob_image_center[1,1]-blob_image_center[0,1], blob_image_center[1,0]-blob_image_center[0,0])

    # theta = 

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, mask_image, color=(255,0,0))

    xw_yw = []

    if(num_blobs == 0):
        # print("No block found!")
        pass
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
