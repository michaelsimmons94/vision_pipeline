import numpy as np
import cv2
import argparse
import imutils
import math
from ColorFilter import ColorFilter


def which_shape(c):
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.035 * peri, True)
        approx=np.squeeze(approx)
        # print 'diffs'
        # if len(approx)>6:
        #     return 'circle'
        # if len(approx)<3:
        #     return shape
        dim=approx.shape[0]
        diffs=np.zeros((dim,dim))
        for idx,x in enumerate(approx):
            temp= np.abs(approx-x)
            temp= np.square(temp)
            # print(temp)
            temp=np.sum(temp,axis=1)
            temp= np.sqrt(temp)
            
            diffs[idx]=temp
        
        upper_triange= np.triu_indices(dim,1)
        
        limit= np.average(diffs[upper_triange])-1.5*np.std(diffs[upper_triange])
        points_to_remove= np.where(diffs[upper_triange]<limit)
        approx_to_remove=upper_triange[1][points_to_remove]
        approx=np.delete(approx, approx_to_remove, 0)
        # printprint approx
        # print np.where(np.logical_and())    
        
        if len(approx) == 3:
            shape = "t"
        elif len(approx) == 4:
            shape = "s"
        else:
            shape = "c"
        # print "shape:",shape
        # print "approx:", approx
        return shape

def get_contours(img_mask):
    _, contours, _ = cv2.findContours(img_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    return contours
def unwrap(point):
    point=np.squeeze(point)
    return (point[0],point[1])
def label(image,colorFilter):
    color_mask=colorFilter.getMask(image)
    contours=get_contours(color_mask)
    # print(type(contours))
    labels=[]
    for c in contours:
        hull = cv2.convexHull(c)
        if cv2.contourArea(hull)>30:

            shape=which_shape(hull)
            
            # print shape
            
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
    
    
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
    help="path to the input image")
args = vars(ap.parse_args())
# load the image and resize it to a smaller factor so that
# the shapes can be approximated better
image = cv2.imread(args["image"])
# image= cv2.imread("headshot.png")

hsv= cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
# print hsv[927,744]
# print hsv[948,719]
# print hsv[985,770]
# print hsv[349,420]



# print hsv[438,613]
# print hsv[438, 617]
# print hsv[355,144]
# print hsv[454,196]
# print(hsv[448,371])
# print(hsv[448,376])
# print(hsv[444,376])
# print hsv[445,358]
lower_red = np.array([168,165,145]) 
upper_red = np.array([173,230,230]) 

lower_blue = np.array([93,110,190]) 
upper_blue = np.array([103,215,255]) 

lower_green = np.array([42,72,160]) 
upper_green = np.array([64,120,215])

redFilter=ColorFilter(lower_red,upper_red,'r')
blueFilter=ColorFilter(lower_blue,upper_blue, 'b')
# output=cv2.bitwise_or(edges,redFilter.getMask(image))
# output=cv2.erode(output,np.ones((3,3), np.uint8),iterations=1)
# output=cv2.medianBlur(output,3)
greenFilter=ColorFilter(lower_green, upper_green, 'g')
# output=blueFilter.getMask(image)

label(image,greenFilter)
label(image,blueFilter)
# labels=label(image,greenFilter)
# output=greenFilter.getMask(image)
# print(type(output))
# for l in labels:
    # print(l)
# output=blueFilter.getMask(image)
# print(hsv[444,351])
# print(output[420,380])
# output = image[420:472,300:380]
# output=blueFilter.getMask(image)
output=image
cv2.imshow("Image", output)
cv2.waitKey(0)