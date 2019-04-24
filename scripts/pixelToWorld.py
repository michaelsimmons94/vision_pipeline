#!/usr/bin/env python
import rospy
import sys
import math
import numpy as np
from vision_pipeline.srv import GetPoint
from geometry_msgs.msg import Point

class Transformer:

    def __init__(self):
        #camera offsets
        # self.c_x_off = -0.01
        # self.c_y_off= 0.28
        # self.c_z_off = 0.01
        # self.angle= -.5
        self.c_x_off = 0.02
        self.c_y_off= 0.49
        self.c_z_off = 0.09
        self.angle= .7
        #matrices
        self.trans_mat=np.eye(4)
        
        self.init_transformation_matrix()
        self.rot_mat= np.zeros((4,4))
        self.init_rotation_matrix()
        rospy.init_node('vision')
        rospy.Service('vision/pixelToWorld', GetPoint, self.getCoordinates)
        rospy.spin()

    def getCoordinates(self, req):
        rospy.wait_for_service('vision/pixelToCamera')
        try:
            pixelToCamera= rospy.ServiceProxy('vision/pixelToCamera', GetPoint)
            resp= pixelToCamera(req.x,req.y)
            cam_point=resp.point
            t_point= self.transform(cam_point)
            print '{},{},{},{},{},{}'.format(cam_point.x, cam_point.y, cam_point.z, t_point.x[0], t_point.y[0], t_point.z[0])
            return t_point
            
        except rospy.ServiceException, e:
            print 'Service call failed: %s'%e
    
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
        # results=np.matmul(self.rot_mat,np.matmul(self.trans_mat,in_vector))
        results=np.matmul(self.trans_mat,np.matmul(self.rot_mat,in_vector))
        msg=Point()
        msg.x=results[0]
        msg.y=results[1]
        msg.z=results[2]
        return msg
if __name__ == "__main__":
    Transformer()

