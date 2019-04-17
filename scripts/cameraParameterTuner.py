#!/usr/bin/env python
import rospy
import sys
import math
import numpy as np
from tqdm import tqdm
from vision_pipeline.srv import GetPoint
from geometry_msgs.msg import Point

class Transformer:

    def __init__(self):
        #camera offsets
        # theta = -math.radians(30)
        self.c_x_off = 0
        self.c_y_off = 0.31
        self.c_z_off = 0.04
        #matrices
        self.trans_mat=np.eye(4)
        self.set_translation_matrix(.03,.31,.04)
        self.rot_mat= np.zeros((4,4))
        self.set_rotation_matrix(30)
        self.loadData()

    
    def set_translation_matrix(self,x_off,y_off,z_off):
        self.trans_mat[0,3]=-x_off
        self.trans_mat[1,3]=-y_off
        self.trans_mat[2,3]=-z_off
    def set_rotation_matrix(self, theta):
        self.rot_mat[0,0]=1
        self.rot_mat[1,1]=math.cos(theta)
        self.rot_mat[1,2]=-math.sin(theta)
        self.rot_mat[2,1]=math.sin(theta)
        self.rot_mat[2,2]=math.cos(theta)
        self.rot_mat[3,3]=1
    def transform(self,x,y,z):
        in_vector=np.reshape(np.array([x,z,y,1]),(4,1))
        results=np.matmul(self.rot_mat,np.matmul(self.trans_mat,in_vector))
        out_x=results[0]
        out_y=results[1]
        return out_x,out_y
    def estimate(self,x,y,z,theta):
        self.set_rotation_matrix(theta)
        self.set_translation_matrix(x,z,y)
        sse=0
        for feature, label in zip(self.features,self.labels):
            feature=self.features[0]
            label=self.labels[0]
            f_x=feature[0]
            f_y=feature[1]
            f_z=feature[2]
            l_x=label[0]
            l_y=label[1]
            # print(f_x,f_y,f_z)
            # print(l_x,l_y)
            out_x,out_y=self.transform(f_x,f_y,f_z)
            # print(out_x,out_y)
            sse+= ((l_x-out_x)**2)+(l_y-out_y)**2
            # print(sse[0])
        return sse
    def sweep(self):
        x_range=np.arange(-.1,.1,.01)
        z_range=np.arange(-.1,.1,.01)
        y_range=np.arange(.15,.45,.01)
        theta_range=np.arange(-1,1,.01)
        bssf_error=float('inf')
        bssf=[]
        for x in tqdm(x_range):
            for y in y_range:
                for z in z_range:
                    for theta in theta_range:
                        error=self.estimate(x,y,z,theta)
                        if error<bssf_error:
                            bssf_error=error
                            bssf=np.copy([x,y,z,theta])
        print(bssf)
        print(bssf_error)
        return bssf
    def loadData(self):
        self.features=np.loadtxt('features.csv', delimiter=',')
        # print(self.features)
        self.labels=np.loadtxt('labels.csv', delimiter=',')
        # print(self.labels)
        # print(self.features.shape)
        # print(self.labels.shape)
if __name__ == "__main__":
    t=Transformer()
    
    t.sweep()

