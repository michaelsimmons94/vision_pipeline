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
        upper_green = np.array([64,120,215])

        #initialize ColorFilters
        self.redFilter=ColorFilter(lower_red,upper_red,'r')
        self.blueFilter=ColorFilter(lower_blue,upper_blue, 'b')
        self.greenFilter=ColorFilter(lower_green, upper_green, 'g')
    
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
            return 'c'
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
            shape = "t"
        elif len(approx) == 4:
            shape = "s"
        else:
            shape = "c"

        return shape

    def label(self,image,colorFilter):
        color_mask=colorFilter.getMask(image)
        _, contours, _ = cv2.findContours(color_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        labels=[]
        for c in contours:
            hull = cv2.convexHull(c)

            if cv2.contourArea(hull)>35:
                shape=self.which_shape(hull)
                M = cv2.moments(hull)
                if M['m00']!=0 and shape!='unidentified':
                    cX = int((M["m10"] / M["m00"]) )
                    cY = int((M["m01"] / M["m00"]) )
                    _label=colorFilter.getLabel()+" "+shape
                    labels.append((_label,cY,cX))
                    cv2.putText(image, _label, (cX+20, cY+20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
                    
                    cv2.drawContours(image, [hull], -1, (0,0,0), 1)
        return labels

    def detectShapes(self,frame):
        self.label(frame,self.blueFilter)
        self.label(frame,self.greenFilter)
        self.label(frame,self.redFilter)    