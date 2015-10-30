#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/PointCloud.h"
#include <iostream>
#include <std_msgs/Bool.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <fstream>
#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
//using namespace
using namespace Eigen;
using namespace std;

//typing
float a;
float b;
float c;
float d;

//can be set to true if you want to skip calibration!
bool rdy=false;
 

//publishers
ros::Publisher pub;
ros::Publisher pub2;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
//wait for calibration. 
if (!rdy){
    return;
}
//Recived message
pcl::PointCloud<pcl::PointXYZRGB>::Ptr not_transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
pcl::fromROSMsg (*cloud_msg,*not_transformed_cloud);

//intial voxel
pcl::PointCloud<pcl::PointXYZRGB>::Ptr inital_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
voxel_grid.setInputCloud (not_transformed_cloud);
voxel_grid.setLeafSize (0.01f, 0.01f, 0.01f);
voxel_grid.filter (*inital_cloud);

// Rotation (and a little bit of translation)
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
Eigen::Affine3f transform = Eigen::Affine3f::Identity();
transform.rotate(AngleAxisf(-acos(abs(b)), Vector3f::UnitX()));
transform.translation() << 0.0f, -d, 0.0f;
pcl::transformPointCloud(*inital_cloud, *transformed_cloud, transform);

// Filter to get walls (for naviation publisher)
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloudWalls (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PassThrough<pcl::PointXYZRGB> pass;
pass.setInputCloud (transformed_cloud);
pass.setFilterFieldName ("y");
pass.setFilterLimits (-0.3f,-0.1f);
pass.filter (*filteredCloudWalls);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr intermediate_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid2;
voxel_grid2.setInputCloud (filteredCloudWalls);
voxel_grid2.setLeafSize (0.1f, 0.1f, 0.1f);
voxel_grid2.filter (*intermediate_cloud);
sensor_msgs::PointCloud2 outputwalls;
pcl::toROSMsg(*intermediate_cloud, outputwalls);
pub2.publish (outputwalls);

//// Filter object for objects
//filter in height
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloudObjects (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PassThrough<pcl::PointXYZRGB> pass2;
pass2.setInputCloud (transformed_cloud);
pass2.setFilterFieldName ("y");
pass2.setFilterLimits (-0.1f,-0.01f);
pass2.filter (*filteredCloudObjects);

//convert to sensor_msgs and Publish the data
sensor_msgs::PointCloud2 cloud_out_objects;
pcl::toROSMsg (*filteredCloudObjects, cloud_out_objects);
pub.publish (cloud_out_objects);
}

//Wait for calibration, when this message is received we wil start generation of the clouds
void run (const std_msgs::Bool& run)
{
rdy=true;
}

int main (int argc, char** argv){

  // Initialize ROS and cretate a handle
  ros::init (argc, argv, "process");
  ros::NodeHandle nh;

  //subscribers
  ros::Subscriber sub=nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
  ros::Subscriber sub2 = nh.subscribe ("/nord/pointcloud/calibration", 1, run);

  //get calibration data
  ifstream indata;
  indata.open("src/nord/nord_pointcloud/data/calibration.txt");
  indata >> a >> b >> c >> d;
  indata.close();

  //publishers
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/nord/pointcloud/processed_objectdebris", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/nord/pointcloud/processed_walls", 1);

  // Spin
  while(ros::ok())
  {
    ros::spinOnce();
  }
}
