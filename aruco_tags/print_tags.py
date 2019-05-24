#!/usr/bin/env python
import cv2
import cv2.aruco as aruco
aruco_dict=aruco.Dictionary_get(aruco.DICT_4X4_250)
# parameters =  aruco.DetectorParameters_create()
# corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
# # print(corners)
# img = aruco.drawDetectedMarkers(img, corners)

for x in range(6,12):
    img= aruco.drawMarker(aruco_dict, x, 200)
    cv2.imwrite('{}.jpg'.format(x),img)