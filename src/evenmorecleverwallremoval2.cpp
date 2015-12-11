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
#include <valarray>
#include <string>
#include "ros/package.h"
#include <sstream>
#include "visualization_msgs/Marker.h"
#include <fstream>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <boost/thread/thread.hpp>

//tweaking?
float withindistance =0.02f;
float thresholdformplane =0.005f; //SHIT IF FUCKED? SET TO 0.01f
ros::Publisher no_wall_pub;
typedef pcl::PointXYZ pointtype;
std::vector<Eigen::VectorXf> v;
int max_planes=11;

void cloud_cb2(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  int counter=0;
  //clear shit
  v.clear();
  //input
  pcl::PointCloud<pointtype>::Ptr wallcloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*cloud_msg, *wallcloud_in); 

  if (wallcloud_in->points.size()==0){
    return;
  }

  while(wallcloud_in->points.size()>20 && ros::ok() && counter<max_planes){
  counter++;
  //segmentation
  //std::cout << wallcloud_in->points.size() << std::endl;
  std::vector<int> inliers;
  Eigen::VectorXf model_coefficients;
  pcl::PointCloud<pointtype>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (wallcloud_in));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
  ransac.setDistanceThreshold (thresholdformplane);
  ransac.computeModel();
  ransac.getInliers(inliers);
  ransac.getModelCoefficients(model_coefficients);
  //std::cout << inliers.size() << std::endl;
  //(*model_p).computeModelCoefficients(inliers, model_coefficients);
  v.push_back(model_coefficients);

  // Create the filtering object
  pcl::PointIndices::Ptr PointIndicesVector (new pcl::PointIndices);
  PointIndicesVector->indices=inliers;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (wallcloud_in);
  extract.setIndices (PointIndicesVector);
  extract.setNegative (true);
  extract.filter (*cloud_f);
  wallcloud_in.swap (cloud_f);
  }

}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  if (v.size()==0){
    return;
}
  //Recive timestamp
  ros::Time timestamp= cloud_msg->header.stamp;

  //Recived message
  pcl::PointCloud<pointtype>::Ptr not_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pointtype>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*cloud_msg, *not_filtered_cloud);

  //std::cout << not_filtered_cloud->points.size() << std::endl;
  std::cout << v.size() << std::endl;
  //evaluate the models from the other pcl
  for (uint i=0; i<v.size(); i++){
  std::vector<int> inliers;
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_c (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (not_filtered_cloud));
  (*model_c).selectWithinDistance(v[i], withindistance, inliers);

  // Create the filtering object
  pcl::PointIndices::Ptr PointIndicesVector (new pcl::PointIndices);
  PointIndicesVector->indices=inliers;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (not_filtered_cloud);
  extract.setIndices (PointIndicesVector);
  extract.setNegative (true);
  extract.filter (*cloud_f);
  not_filtered_cloud.swap (cloud_f);

  }
  //std::cout << not_filtered_cloud->points.size() << std::endl;

  //publish!
  sensor_msgs::PointCloud2 cloud_out_objects;
  cloud_out_objects.header.stamp=timestamp;
  pcl::toROSMsg (*not_filtered_cloud, cloud_out_objects);
  no_wall_pub.publish (cloud_out_objects);  
}

int main (int argc, char** argv){
  // Initialize ROS and cretate a handle
  ros::init (argc, argv, "evenmorecleverwallremoval2");
  ros::NodeHandle nh;

  //subscribers
  ros::Subscriber sub =nh.subscribe ("/nord/pointcloud/processed", 1, cloud_cb);
  ros::Subscriber sub2=nh.subscribe("/nord/pointcloud/walls", 1, cloud_cb2);


  //publishers
  no_wall_pub = nh.advertise<sensor_msgs::PointCloud2>("/nord/pointcloud/no_wall", 1);
  ros::spin();
}