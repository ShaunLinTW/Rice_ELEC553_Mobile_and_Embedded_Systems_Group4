'''
ELEC 424/553
Final Project
Authors: Eric Lin(el38), Shaun Lin(hl116), Yen-Yu Chien (yc185), Saif Khan (sbk7)
'''

import cv2

# initialize the camera
cam = cv2.VideoCapture(2)
ret, image = cam.read()

if ret:
    cv2.imwrite('/home/debian/Group4/src/opencv/SnapshotTest.jpg',image)
cam.release()