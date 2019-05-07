#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "vision_pipeline/PixelToPoint.h"
#include <boost/shared_ptr.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

bool getPoint(vision_pipeline::PixelToPoint::Request  &req, vision_pipeline::PixelToPoint::Response &res)
{
    boost::shared_ptr<sensor_msgs::PointCloud2 const> sharedPtr;
    sharedPtr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect2/sd/points", ros::Duration(10));
    
    if (sharedPtr == NULL)
    {
        std::cout<<"No point clound messages received\n";
        return false;
    }
    else
    {
        int i = (req.x) + (req.y)*(sharedPtr->width);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl:fromROSMsg(*sharedPtr,cloud);
        
        res.point.x=cloud.points[i].x;
        res.point.y=cloud.points[i].y;
        res.point.z=cloud.points[i].z;
        return true;
    }

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pixel_to_camera");
  ros::NodeHandle nh;
  ros::ServiceServer srv= nh.advertiseService("vision/pixelToCamera",getPoint);
  std::cout<<"pixel to camera ready\n";
  ros::spin();
  return 0;
}