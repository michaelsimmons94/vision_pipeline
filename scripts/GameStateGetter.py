#!/usr/bin/env python
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image
from sklearn.cluster import KMeans
import std_srvs.srv
class GameState:
    def __init__(self):
        rospy.init_node('GameState')
        self.bridge = CvBridge()
        self.k=10
        self.red = np.array([217,65,122])
        # purple = np.array([126,117,113])
        self.blue = np.array([69,221,241])
        self.green = np.array([154,207,156])
        red=self.red
        blue=self.blue
        green=self.green
        self.topic='/kinect2/sd/image_color_rect'
        # rospy.Subscriber(self.topic, Image,self.emptyCallback,queue_size=1)
        self.color_dict={'red':red,'blue':blue, 'green':green}
        # 'purple':purple
        self.kernel = np.ones((3,3),np.uint8)
        self.gamestate=np.zeros((3,3))
    def getFrame(self):
        frame=rospy.wait_for_message(self.topic,Image, timeout=1)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(frame, "bgr8")
           
            
            rgb=cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
            return rgb
            # return output_img
        except CvBridgeError as e:
            print(e)
    def quantize_img(self,img):
        (h,w) = img.shape[:2]
        img=cv2.cvtColor(img,cv2.COLOR_RGB2LAB)
        img= img.reshape((img.shape[0]*img.shape[1],3))
        clusters=KMeans(n_clusters=self.k)
        labels=clusters.fit_predict(img)
        quant=clusters.cluster_centers_.astype("uint8")[labels]
        #reshape
        quant = quant.reshape((h,w,3))
        img = img.reshape((h,w,3))
        #convert
        quant = cv2.cvtColor(quant, cv2.COLOR_LAB2RGB)
        img = cv2.cvtColor(img, cv2.COLOR_LAB2RGB)
        return quant, clusters.cluster_centers_
    def setColors(self,centroids):
        centroids=centroids.astype("uint8")
        colors=np.array([centroids])
        
        colors=cv2.cvtColor(colors,cv2.COLOR_LAB2RGB)
        color_vector=np.reshape(colors,(self.k,3))
        for col in self.color_dict:
            sub=np.tile(self.color_dict[col],(self.k,1))
            
            dist=np.linalg.norm((color_vector-sub)[:,1:],axis=1)
            
            idx=np.argmin(dist)
            if dist[idx]<20:
                self.color_dict[col]=color_vector[idx]
        return colors

    def getMask(self,quantized_img,color_name):
        color=self.color_dict[color_name]
        mask=cv2.inRange(quantized_img,color,color)
        return mask

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
        return img[275:380,130:330]

    def emptyCallback(self,msg):
        pass
    def resetColorDict(self):
        self.color_dict['green']=self.green
        self.color_dict['red']=self.red
        self.color_dict['blue']=self.blue
    def getGameState(self):
        
        img=self.getFrame()
        img=self.crop(img)
        q_img, centroids=self.quantize_img(img)
        self.resetColorDict()
        colors=self.setColors(centroids)
        blue_mask=self.getMask(q_img,'blue')
        green_mask=self.getMask(q_img,'green')
        red_mask=self.getMask(q_img,'red')
        self.gamestate=np.zeros((3,3))
        self.getShapes(green_mask,'green',q_img)
        self.getShapes(red_mask,'red',q_img)
        out=self.getShapes(blue_mask,'blue',q_img)

        print(self.gamestate)
        # self.displayMask(blue_mask)
        # self.display(out)
        return self.gamestate

    def which_shape(self,c):
        '''
        Returns the shape of the object with c=Circle t=Triangle and s=Square
        '''
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.035 * peri, True)
        approx=np.squeeze(approx)
        if len(approx)>5:
            return 'circle'
        elif len(approx)<3:
            return shape
        #eliminate points that are too close together
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
            shape = "triangle"
        elif len(approx) == 4:
            shape = "square"
        else:
            shape = "circle"

        return shape

    def fill_game_state(self,shape_name,color_name):
        row=0
        col=0
        if shape_name=='square':
            row=2
        elif shape_name=='triangle':
            row=1
        if color_name=='red':
            col=2
        elif color_name=='blue':
            col=1
        self.gamestate[row][col]=1


    def getShapes(self, mask, color_name,img):
        _, contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        labels=[]
        for c in contours:
            hull = cv2.convexHull(c)

            shape=self.which_shape(hull)
            self.fill_game_state(shape,color_name)
            M = cv2.moments(hull)
            if M['m00']!=0 and shape!='unidentified':
                cX = int((M["m10"] / M["m00"]) )
                cY = int((M["m01"] / M["m00"]) )
                _label=shape
                
                # cv2.putText(img, _label, (cX-10, cY+20), cv2.FONT_HERSHEY_SIMPLEX,
                #     0.5, (0,0,0), 1)
                
                cv2.drawContours(img, [hull], -1, (255,0,255), 1)
        return img

    def callback(self,req):
        self.getGameState()
        return std_srvs.srv.EmptyResponse

def main():
    # rospy.sleep(3)
    gs=GameState()
    # gs.getGameState()
    rospy.Service('gamestate',std_srvs.srv.Empty, gs.callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()