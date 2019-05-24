#!/usr/bin/env python
import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vision_pipeline.srv import GetBox
import rospy
import sys

class aruco_detector:

    def __init__(self):
        self.bridge = CvBridge()
        self.hd='/kinect2/hd/image_color_rect'
        self.qhd='/kinect2/qhd/image_color_rect'
        self.sd="/kinect2/sd/image_color_rect"
        self.aruco_dict=aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.parameters =  aruco.DetectorParameters_create()
        # rospy.Service('vision/getbox',GetBox,self.get_box_CB)
        #table from l-> r, top-> bottom:
        #0,1,2,3
        #4,6,5,9
        self.left_box=[0,1,4,6]
        self.center_box=[1,2,6,5]
        self.right_box=[2,3,5,9]
        
    def get_box_CB(self, img):
        gray= cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        points = self.getCenterPoints(corners,ids)
        pts_dst = np.array([[0, 0],[150, 0],[150, 270],[0, 270]])
        pts_dst_shape= (150,270)
        try:
            pts_src=self.getCenterbox(img,points)
            pts_dst= np.array([[0,0],[360,0],[360,270],[0,270]])
            pts_dst_shape =(360,270)
            inv_h, _ =cv2.findHomography(pts_dst,pts_src)
            h, status = cv2.findHomography(pts_src, pts_dst)
            im_out = cv2.warpPerspective(img, h, pts_dst_shape)
            # return img and inverse homography to help with relocalizing the image
            return im_out, inv_h
                
        except Exception:
            print('Not all Aruco markers detected')
            return img

    def getCenterPoints(self,corners, ids):
        zipped=zip(corners,ids)
        zipped.sort(key=lambda t:t[1])
        temp={}
        for c, i in zipped:
            coords=np.around(np.average(np.squeeze(c[0]),axis=0)).astype(int).tolist()
            temp[i[0]]=coords
        return temp
    def getPoints(self,corners, ids):
        zipped=zip(corners,ids)
        zipped.sort(key=lambda t:t[1])
        temp={}
        for c, i in zipped:
            coords=np.squeeze(c[0]).astype(int).tolist()
            temp[i[0]]=coords
        return temp
    
    def get_box(self,img, box,points):
        tl=points[box[0]]
        tr=points[box[1]]
        bl=points[box[2]]
        br=points[box[3]]
        pts = np.array([tl,tr,br,bl], np.int32)
        # temp = pts.reshape((-1,1,2))
        # cv2.polylines(img,[temp],True,self.magenta,2)
        return pts

    def valid_box(self, which_box, points):
        return all(elem in points.keys()  for elem in which_box)

    def getLeftbox(self,img,points):
        return self.get_box(img,self.left_box,points)
        
    def getRightbox(self,img,points):
        return self.get_box(img,self.right_box,points)
        
    def getCenterbox(self,img,points):
        return self.get_box(img,self.center_box,points)

# def main(args):
#   aruco_detector()
#   rospy.init_node('aruco_detector', anonymous=True)
#   try:
#     rospy.spin()
#   except KeyboardInterrupt:
#     print("Shutting down")
#   cv2.destroyAllWindows()


# if __name__ == '__main__':
#     main(sys.argv)      
