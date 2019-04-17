#!/usr/bin/env python
import numpy as np
import math
import rospy
from geometry_msgs.msg import Point

class Transformer:

    def __init__(self):
        #camera offsets
        self.angle = -math.radians(30)
        self.c_x_off = -.03
        self.c_y_off = 0.31
        self.c_z_off = 0.04
        #matrices
        self.trans_mat=np.eye(4)
        self.init_transformation_matrix
        self.rot_mat= np.zeros((4,4))
        self.init_rotation_matrix()
        rospy.init_node('camera_to_world',anonymous=True)
        rospy.Service('vision/cameraToWorld', Point, self.transform)
        rospy.spin()

    def init_transformation_matrix(self):
        self.trans_mat[0,3]=-self.c_x_off
        self.trans_mat[1,3]=-self.c_y_off
        self.trans_mat[2,3]=-self.c_z_off
    def init_rotation_matrix(self):
        self.rot_mat[0,0]=1
        self.rot_mat[1,1]=math.cos(self.angle)
        self.rot_mat[1,2]=-math.sin(self.angle)
        self.rot_mat[2,1]=math.sin(self.angle)
        self.rot_mat[2,2]=math.cos(self.angle)
        self.rot_mat[3,3]=1
    def transform(self,pt):
        in_vector=np.reshape(np.array([pt.x,pt.z,pt.y,1]),(4,1))
        results=np.matmul(self.rot_mat,np.matmul(self.trans_mat,in_vector))
        msg=Point()
        msg.x=results[0]
        msg.y=results[1]
        msg.x=results[2]
        
if __name__ =='__main__':
    Transformer()