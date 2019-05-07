#!/usr/bin/env python
import numpy as np
import rospy
import cv2
import json
from matplotlib.pyplot import imsave
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image
from sklearn.cluster import KMeans,MeanShift
from vision_pipeline.srv import *


class Perception:

    def __init__(self):
        rospy.init_node('perception')
        self.bridge = CvBridge()
        self.k=10
        self.Color={'red':0,'blue':1,'green':2}
        self.Shape={'square':0,'triangle':1, 'circle':2}
        self.red=[119,191,137]
        self.blue=[194,90,152]
        self.green=[200,97,103]
        red=self.red
        blue=self.blue
        green=self.green
        self.y_off=275
        self.x_off=130
        self.topic='/kinect2/sd/image_color_rect'
        #auto update messages so we get the most recent message
        rospy.Subscriber(self.topic, Image,self.emptyCallback,queue_size=1)
        self.color_dict={self.Color['red']:red, self.Color['green']:green, self.Color['blue']:blue}
        self.kernel = np.ones((3,3),np.uint8)
        self.gamestate=np.zeros((3,3))
        self.block_locs=np.zeros((3,3,2))
    

    def getFrame(self):
        frame=rospy.wait_for_message(self.topic,Image, timeout=1)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(frame, "bgr8")
           
            
            rgb=cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
            return rgb

        except CvBridgeError as e:
            print(e)

    def quantize_img(self,img):
        (h,w) = img.shape[:2]
        img=cv2.cvtColor(img,cv2.COLOR_RGB2LAB)
        img= img.reshape((h*w,3))
        clusters=KMeans(n_clusters=self.k)
        labels=clusters.fit_predict(img)
        quant=clusters.cluster_centers_.astype("uint8")[labels]
        centroids=clusters.cluster_centers_
        quant = quant.reshape((h,w,3))
        return quant, centroids

    def setColors(self,centroids):
        self.resetColorDict()
        centroids=centroids.astype("uint8")
        colors=np.array([centroids])

        color_vector=np.reshape(colors,(self.k,3))
        lum_invariant_colors=color_vector[:,1:].astype(int)
        for col in self.color_dict:
            sub=self.color_dict[col][1:].astype(int)
            dist=np.linalg.norm(np.subtract(lum_invariant_colors,sub),axis=1)
            idx=np.argmin(dist)
            if dist[idx]<20:
                self.color_dict[col]=color_vector[idx]
        return colors

    def getMask(self,quantized_img,color_name):
        color=self.color_dict[color_name]
        mask=cv2.inRange(quantized_img,color,color)
        return mask

    def displayLAB(self,img):
        bgr=cv2.cvtColor(img,cv2.COLOR_LAB2BGR)
        cv2.imshow('img',bgr)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def display(self,img):
        bgr=cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        cv2.imshow('img',bgr)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def displayMask(self,mask):
        cv2.imshow('img',mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def crop(self,img):
        return img[self.y_off:self.y_off+105, self.x_off:self.x_off+200]

    def emptyCallback(self,msg):
        pass

    def resetColorDict(self):
        self.color_dict[self.Color['green']]=self.green
        self.color_dict[self.Color['red']]=self.red
        self.color_dict[self.Color['blue']]=self.blue

    def readBoard(self):
        
        img=self.getFrame()
        # normal_image=cv2.imread('./src/vision_pipeline/test_imgs/close_img.png')
        # self.display(normal_image)
        # img=cv2.cvtColor(normal_image,cv2.COLOR_BGR2RGB)
        # self.display(img)
        img=self.crop(img)
        
        q_img, centroids=self.quantize_img(img)
        colors=self.setColors(centroids)
        
        self.gamestate=np.zeros((3,3))
        for color in self.Color.values():
            self.getShapes(color,q_img)
    def blockToPixelCB(self,req):
        try:
            color=self.Color[req.color.lower()]
        except:
            print('color doesn\'t exist')
        try:
            shape=self.Shape[req.shape.lower()]
        except:
            print('shape doesn\'t exist')
        y,x=self.getBlockLoc(color,shape)
        res=BlockToPixelResponse()
        res.x=x
        res.y=y
        return res

    def getBlockLoc(self, color,shape):
        
        self.readBoard()
        if self.gamestate[shape,color]:
            y=self.y_off + self.block_locs[shape,color,0]
            x=self.x_off + self.block_locs[shape,color,1]
        else:
            y=-1
            x=-1
        return y,x

    def gameStateCB(self,req):
        gamestate=self.getGameState()
        gamestate=gamestate.flatten().tolist()
        return GameStateResponse(gamestate)

    def getGameState(self):
        
        self.readBoard()
        print(self.gamestate.astype(int))
        
        return self.gamestate

    def which_shape(self,c):
        '''
        Returns the shape of the object with c=Circle t=Triangle and s=Square
        '''
        # approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.035 * peri, True)
        approx=np.squeeze(approx)
        if len(approx)>5:
            return self.Shape['circle']
        #ignore cases that are lines/not polygons
        if len(approx)<=2:
            return -1
        # #eliminate points that are too close together
        dim=approx.shape[0]
        diffs=np.zeros((dim,dim))
        for idx,x in enumerate(approx):
            temp= np.abs(approx-x)
            temp= np.square(temp)
            temp=np.sum(temp,axis=1)
            temp= np.sqrt(temp)
            diffs[idx]=temp
        
        upper_triange= np.triu_indices(dim,1)
        
        limit= np.average(diffs[upper_triange])-1.5*np.std(diffs[upper_triange])
        points_to_remove= np.where(diffs[upper_triange]<limit)
        approx_to_remove=upper_triange[1][points_to_remove]
        approx=np.delete(approx, approx_to_remove, 0)  
        #determine shape
        if len(approx) == 3:
            return self.Shape['triangle']
        elif len(approx) == 4:
            return self.Shape['square']
        else:
            return self.Shape['circle']

    def getShapes(self, color,img):
        
        mask=self.getMask(img, color)
        _, contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            hull = cv2.convexHull(c)

            shape=self.which_shape(hull)
            if shape>=0:
                self.gamestate[shape][color]=1
                M = cv2.moments(hull)
                if M['m00']!=0:
                    cX = int((M["m10"] / M["m00"]) )
                    cY = int((M["m01"] / M["m00"]) )
                    self.block_locs[shape,color]=[cY,cX]
                    # cv2.drawContours(img, [hull], -1, (255,0,255), 1)
        # return img


    

def main():
    perception= Perception()
    rospy.Service('vision/gamestate',GameState, perception.gameStateCB)
    rospy.Service('vision/blockToPixel',BlockToPixel,perception.blockToPixelCB)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    # cv2.destroyAllWindows()
if __name__ == '__main__':
    main()