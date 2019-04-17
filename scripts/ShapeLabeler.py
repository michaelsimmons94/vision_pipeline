#!/usr/bin/env python
from __future__ import print_function

# import roslib
# roslib.load_manifest('my_package')
import numpy as np
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from detector import Detector
from vision_pipeline.msg import *

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("detected_shapes",Image, queue_size=10)
    self.bridge = CvBridge()
    hd='/kinect2/hd/image_color_rect'
    sd="/kinect2/sd/image_color_rect"
    self.shape_locs_pub = rospy.Publisher("shape_locs", Block2D, queue_size=10)
    self.image_sub = rospy.Subscriber(sd,Image,self.callback)
    
    self.detector= Detector()

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      output_img=np.copy(cv_image)
    except CvBridgeError as e:
      print(e)
    t=self.detector.detectShapes(cv_image,output_img)
    if t:
      print(t)
      print(output_img.shape)
      t=t[0]
      msg=Block2D()
      msg.color=t[0]
      msg.shape=t[1]
      msg.y=t[2]
      msg.x=t[3]
      self.shape_locs_pub.publish(msg)
    cv2.imshow("Shape Detector", output_img)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)