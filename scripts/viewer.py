#!/usr/bin/env python
import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vision_pipeline.srv import GetBox
import rospy
import sys

class viewer:
    def __init__(self):
        self.image_pub = rospy.Publisher("aruco",Image, queue_size=1)
        self.bridge = CvBridge()
        self.hd='/kinect2/hd/image_color_rect'
        self.qhd='/kinect2/qhd/image_color_rect'
        self.sd="/kinect2/sd/image_color_rect"
        self.image_sub = rospy.Subscriber(self.qhd,Image,self.callback)
        self.aruco_dict=aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.parameters =  aruco.DetectorParameters_create()
        # rospy.Service('vision/getbox',GetBox,self.get_box_CB)
        #table from l-> r, top-> bottom:
        #0,1,2,3
        #4,6,5,9
        self.left_box=[0,1,4,6]
        self.center_box=[1,2,6,5]
        self.right_box=[2,3,5,9]
        self.magenta=(255,0,255)
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            img=np.copy(cv_image)

        except CvBridgeError as e:
            print(e)
        gray= cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        points = self.getCenterPoints(corners,ids)
        aruco.drawDetectedMarkers(img, corners, ids)

        self.drawAllBoxes(img,points)
        cv2.imshow("Viewer", img)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
    def getCenterPoints(self,corners, ids):
        zipped=zip(corners,ids)
        zipped.sort(key=lambda t:t[1])
        temp={}
        for c, i in zipped:
            coords=np.around(np.average(np.squeeze(c[0]),axis=0)).astype(int).tolist()
            temp[i[0]]=coords
        return temp
    def drawBox(self,img, box,points):
        try:
            tl=points[box[0]]
            tr=points[box[1]]
            bl=points[box[2]]
            br=points[box[3]]
            pts = np.array([tl,tr,br,bl], np.int32)
            temp = pts.reshape((-1,1,2))
            cv2.polylines(img,[temp],True,self.magenta,2)
        #TODO: make it so it can determine which marker and box is obscured
        except Exception:
            pass
    def drawAllBoxes(self,img,points):
        self.drawBox(img,self.left_box,points)
        self.drawBox(img,self.center_box,points)
        self.drawBox(img,self.right_box,points)
def main(args):
    viewer()
    rospy.init_node('viewer', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)      