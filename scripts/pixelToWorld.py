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
        self.c_x_off = 0
        # self.c_y_off = 0.39
        self.c_y_off= 0.356
        self.c_z_off = 0.05
        # self.angle = -0.73
        self.angle= -.48
        #matrices
        self.trans_mat=np.eye(4)
        
        self.init_transformation_matrix()
        # print(self.trans_mat)
        self.rot_mat= np.zeros((4,4))
        self.init_rotation_matrix()
        # print(self.rot_mat)
        rospy.init_node('vision')
        rospy.Service('vision/pixelToWorld', GetPoint, self.getCoordinates)
        rospy.spin()

    def getCoordinates(self, req):
        rospy.wait_for_service('vision/pixelToCamera')
        try:
            pixelToCamera= rospy.ServiceProxy('vision/pixelToCamera', GetPoint)
            resp= pixelToCamera(req.x,req.y)
            cam_point=resp.point
            print 'camera points:',cam_point.x,cam_point.y,cam_point.z
            return self.transform(cam_point)
            
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
        results=np.matmul(self.rot_mat,np.matmul(self.trans_mat,in_vector))
        # results=np.matmul(self.trans_mat,np.matmul(self.rot_mat,in_vector))
        print 'world points:',results
        msg=Point()
        msg.x=results[0]
        msg.y=results[1]
        msg.z=results[2]
        return msg
if __name__ == "__main__":
    Transformer()

