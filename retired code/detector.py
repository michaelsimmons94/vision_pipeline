import numpy as np
import cv2
from ColorFilter import ColorFilter


class Detector:

    def __init__(self):
        #initialize hsv colorbands
        lower_red = np.array([168,165,145]) 
        upper_red = np.array([173,230,230])  
        lower_blue = np.array([93,110,190]) 
        upper_blue = np.array([103,215,255])
        lower_green = np.array([42,72,160]) 
        upper_green = np.array([67,120,225])
        lower_orange= np.array([12,78,227])
        upper_orange= np.array([22,168,255])
        lower_purple=np.array([133,15,76])
        upper_purple=np.array([160,75,150])
        #initialize ColorFilters
        self.redFilter=ColorFilter(lower_red,upper_red,'red')
        self.blueFilter=ColorFilter(lower_blue,upper_blue, 'blue')
        self.greenFilter=ColorFilter(lower_green, upper_green, 'green')
        self.purpleFilter=ColorFilter(lower_purple,upper_purple, 'purple')
        self.color_dict={'red':(255,0,0),'green':(0,255,0),'blue':(255,0,0),'purple':(255,0,255)}
        # self.orangeFilter=ColorFilter(lower_orange,upper_orange, 'org')
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
    def getPlayAreaContour(self,img, hd=False):
        img_y, img_x, img_c=img.shape
        pts=[]
        if hd:
            pts = np.array([[0,img_y-1],[136,872],[1269,848],[1362,img_y-1]], np.int32)
        else:
            pts = np.array([[0,382],[50,275],[413,290],[448,382]], np.int32)
        pts = pts.reshape((-1,1,2))
        return pts

    def draw_play_area(self,img):
        ctr=self.getPlayAreaContour(img)
        cv2.polylines(img,[ctr],True,(0,0,255))
        
    def isInPlayArea(self, img,point):
        ctr=self.getPlayAreaContour(img)
        return cv2.pointPolygonTest(ctr,point,False)

    def label(self,image,colorFilter,output):
        color_mask=colorFilter.getMask(image)
        color_tup=self.color_dict[colorFilter.label]
        _, contours, _ = cv2.findContours(color_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        labels=[]
        for c in contours:
            hull = cv2.convexHull(c)

            if cv2.contourArea(hull)>40:
                shape=self.which_shape(hull)
                M = cv2.moments(hull)
                if M['m00']!=0 and shape!='unidentified':
                    cX = int((M["m10"] / M["m00"]) )
                    cY = int((M["m01"] / M["m00"]) )
                    if self.isInPlayArea(image,(cX,cY))>0:
                        # _label=colorFilter.getLabel()+" "+shape
                        _label=shape
                        labels.append((shape,colorFilter.getLabel(),cY,cX))
                        
                        cv2.putText(output, _label, (cX-10, cY+20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, color_tup, 1)
                        
                        cv2.drawContours(output, [hull], -1, (0,0,0), 1)
        self.draw_play_area(output)
        return labels

    def detectShapes(self,frame,output):
        labels=[]
        labels+=self.label(frame,self.blueFilter,output)
        labels+=self.label(frame,self.greenFilter,output)
        # self.label(frame,self.redFilter)  
        # self.label(frame,self.orangeFilter)
        labels+=self.label(frame,self.purpleFilter,output)
        return labels
    