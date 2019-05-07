#!/usr/bin/env python
from __future__ import print_function
import rospy
import numpy as np

import tf_conversions
import tf2_ros
import tf2_geometry_msgs as tf2

from vision_pipeline.srv import PointToPoint,PointToPointResponse
class Transformer:

  def __init__(self):
    rospy.init_node('cameraToPoint', anonymous=True)
    self.tf_buffer= tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tf_buffer)
    
    rospy.Service('vision/cameraToPoint', PointToPoint,self.callBack)
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")

  def reqToVector(self,req):
    p=req.point
    return np.array([p.x,p.y,p.z,1])
  def getTransformation(self):
    
    try:
        self.tf_buffer.can_transform("base", "kinect2_ir_optical_frame", rospy.Time(), rospy.Duration(10.0))
        trans = self.tf_buffer.lookup_transform('base','kinect2_ir_optical_frame',rospy.Time(), rospy.Duration(1.0))
        t=trans.transform.translation
        r=trans.transform.rotation
        quaternion=np.array([r.x,r.y,r.z,r.w])
        mat=tf_conversions.transformations.quaternion_matrix(quaternion)
        mat[0,3]+=t.x
        mat[1,3]+=t.y
        mat[2,3]+=t.z
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
        rospy.logwarn(ex)
        return
    
    # print(type(trans))
    
    return mat

  def callBack(self,req):
    vec = self.reqToVector(req)
    mat=self.getTransformation()
    vec= mat.dot(vec)
    res=PointToPointResponse()
    res.point.x=vec[0]
    res.point.y=vec[1]
    res.point.z=vec[2]
    return res

Transformer()