#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <fstream>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <nord_messages/PoseEstimate.h>
#include <pcl/filters/conditional_removal.h>
#include <iostream>
#include <string>
#include "ros/package.h"
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <sstream>
#include <pcl/common/centroid.h>
#include <nord_messages/Coordinate.h>
#include <nord_messages/CoordinateArray.h>

typedef pcl::PointXYZ pointtype;

pcl::PointCloud<pointtype>::Ptr not_transformed_cloud (new pcl::PointCloud<pointtype>());
std::string name;
bool pressed;
uint recorded;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  //Recive timestamp
  ros::Time timestamp= cloud_msg->header.stamp;

  //Recived message
  pcl::fromROSMsg (*cloud_msg,*not_transformed_cloud);
 }

 void centroids_cb(const nord_messages::CoordinateArray::ConstPtr& centroids){
 if (centroids->data.size() != 1){
  return;
 }

if(!pressed)
  {
    return;
  }

 pressed=false;
 pcl::PointCloud <pcl::VFHSignature308> point;
 point.push_back(pcl::VFHSignature308());
for(uint k=0;k<308;k++){
 point[0].histogram[k]=centroids->data[0].features.vfh[k];
}
 std::stringstream out;
 out << ros::package::getPath("nord_pointcloud") << "/data/" << name <<  recorded << ".pcd";
  pcl::io::savePCDFileASCII (out.str(), point);
  std::cout << "Saved " << out.str() << std::endl;
  recorded++;
}

int main (int argc, char** argv){
  // Initialize ROS and cretate a handle
  ros::init (argc, argv, "gatherdata");
  ros::NodeHandle nh;

  name=std::string(argv[1]);
  ros::Subscriber sub =nh.subscribe ("nord/pointcloud/no_wall", 1, cloud_cb);
  //ros::Subscriber sub =nh.subscribe ("nord/pointcloud/processed", 1, cloud_cb);
  ros::Subscriber sub2 =nh.subscribe ("nord/pointcloud/centroids", 1, centroids_cb);

  ros::Rate R(10);
  while (ros::ok()){
    std::cin.get();
    pressed=true;

    ros::spinOnce();
    R.sleep();


  }
}