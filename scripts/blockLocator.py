#!/usr/bin/env python
import rospy
from vision_pipeline.srv import BlockToPixel,BlockToPoint,PixelToPoint, PointToPoint
class Locator:
    def __init__(self):
        self.blockToPixel=rospy.ServiceProxy('vision/blockToPixel',BlockToPixel)
        self.pixelToCam=rospy.ServiceProxy('vision/pixelToCamera', PixelToPoint)
        self.camToRobot=rospy.ServiceProxy('vision/cameraToPoint', PointToPoint)

    def getLoc(self,req):
        pixel=self.blockToPixel(req.color,req.shape)
        cam = self.pixelToCam(pixel.x,pixel.y)
        robot=self.camToRobot(cam.point)
        return robot.point

if __name__ == "__main__":
    rospy.init_node('blockLocator',anonymous=False)
    locator=Locator()
    rospy.Service('vision/locateBlock', BlockToPoint,locator.getLoc)
    rospy.spin()