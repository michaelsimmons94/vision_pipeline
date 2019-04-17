import numpy as np
import cv2
import matplotlib.pyplot as plt

img=cv2.imread('../test_imgs/rd5.png')
area_mask=np.zeros((img.shape[0],img.shape[1]))
print(img.shape)
area_mask[472:,30:895]=1
cv2.circle(area_mask,(400,465),20,(255,0,0),-1)
cv2.bitwise_and(img,img,)
# img[472:,30:895]=[255,0,0]
print(area_mask.shape)
# out=np.logical_and(img,area_mask)
cv2.imshow('img',area_mask)
# cv2.imshow('img',img)
cv2.waitKey(0)