import numpy as np
import cv2


class ColorFilter:

    def __init__(self, lower,upper, label):
        '''
        lower and upper are np.arrays of hsv values ranging from 0-255
        label is the text label you'd like to add
        '''
        self.label=label
        self.lower=lower
        self.upper=upper
        self.kernel=np.ones((5,5),np.uint8)

    def getLabel(self):
        return self.label
    def getMask(self,image):
        '''
        returns a color mask of the image
        '''

        img=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img, self.lower, self.upper)
        mask = cv2.dilate(mask,self.kernel,iterations = 1)
        mask= cv2.erode(mask,self.kernel,iterations=1)
        return mask