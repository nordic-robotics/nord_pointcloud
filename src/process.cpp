#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include "ros/package.h"

//parameters for tweeking
float upper_limit= 0.1;
float voxel_size=0.0075;
typedef pcl::PointXYZ pointtype;

//typing
float a;
float b;
float c;
float d;

//can be set to true if you want to skip calibration!
bool rdy=true;

//droprate
uint k=1;
//state
uint j=0;

//publishers
ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){


//wait for calibration. 
if (!rdy){
    return;
}
j++;
if (j==k)
{
  j=0;
}
else
{
  return;
}

//Recived message
pcl::PointCloud<pointtype>::Ptr not_transformed_cloud(new pcl::PointCloud<pointtype>());
pcl::fromROSMsg (*cloud_msg, *not_transformed_cloud);

//intial voxel
pcl::PointCloud<pointtype> inital_cloud;
pcl::VoxelGrid<pointtype> voxel_grid;
voxel_grid.setInputCloud (not_transformed_cloud);
voxel_grid.setLeafSize (voxel_size, voxel_size, voxel_size);
voxel_grid.filter (inital_cloud);

// Rotation (and a little bit of translation)
pcl::PointCloud<pointtype>::Ptr transformed_cloud(new pcl::PointCloud<pointtype>());
Eigen::Affine3f transform = Eigen::Affine3f::Identity();

transform.rotate(Eigen::AngleAxisf(-std::acos(std::abs(b)), Eigen::Vector3f::UnitX()));
pcl::PointCloud<pointtype>::Ptr mellan_cloud(new pcl::PointCloud<pointtype>());

pcl::transformPointCloud(inital_cloud, *mellan_cloud, transform);

transform = Eigen::Affine3f::Identity();
transform.translation() << 0.0f, -d, 0.0f;
pcl::transformPointCloud(*mellan_cloud, *transformed_cloud, transform);

//// Filter object for objects (is a filter in height removes NaN). 
pcl::PointCloud<pointtype> filteredCloudObjects;
pcl::PassThrough<pointtype> pass;
pass.setInputCloud (transformed_cloud);
pass.setFilterFieldName ("y");
pass.setFilterLimits (-upper_limit,-0.01f);
pass.filter (filteredCloudObjects);

//publish!
sensor_msgs::PointCloud2 cloud_out_objects;
cloud_out_objects.header = cloud_msg->header;
pcl::toROSMsg (filteredCloudObjects, cloud_out_objects);
pub.publish (cloud_out_objects);
}

//Wait for calibration, when this message is received we wil start generation of the clouds
void run (const std_msgs::Bool& run)
{

//set so that we will start process data in the other callback
rdy=true;

//get calibration data
std::ifstream indata;
indata.open((ros::package::getPath("nord_pointcloud") + "/data/calibration.txt").c_str());
indata >> a >> b >> c >> d;
indata.close();
std::cout<< "Calibration received" << std::endl;

}
int main (int argc, char** argv){

  // Initialize ROS and cretate a handle
  ros::init (argc, argv, "process");
  ros::NodeHandle nh;

  //subscribers
  ros::Subscriber sub=nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
  ros::Subscriber sub2 = nh.subscribe ("/nord/pointcloud/calibration", 1, run);

  //publishers
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/nord/pointcloud/processed", 1);

  ros::spin();
}
