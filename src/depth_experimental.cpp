#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "vision_pipeline/Snap.h"
#include <boost/shared_ptr.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/transforms.h>

// #include <pcl/visualization/cloud_viewer.h>

bool snap(vision_pipeline::Snap::Request  &req, vision_pipeline::Snap::Response &res)
{
    boost::shared_ptr<sensor_msgs::PointCloud2 const> sharedPtr;
    sharedPtr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect2/qhd/points", ros::Duration(10));
    
    if (sharedPtr == NULL)
    {
        std::cout<<"No point clound messages received\n";
        return false;
    }
    else
    {
        
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;
        pcl::fromROSMsg(*sharedPtr,cloud);
        pcl::visualization::CloudViewer viewer("Cloud Viewer");
        viewer.showCloud(cloud);
        while(!viewer.wasStopped()){}
        // pcl::io::savePCDFileASCII (req.filename+".pcd", cloud);
        // std::cout<<"Point Cloud Saved\n";
        res.success=true;
        return true;
    }

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_experimental");
  ros::NodeHandle nh;
  ros::ServiceServer srv= nh.advertiseService("vision/snapPointCloud", snap);
  std::cout<<"depth_experiment ready\n";
  ros::spin();
  return 0;
}