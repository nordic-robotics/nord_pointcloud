#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <std_msgs/Bool.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <iostream>
#include <fstream>
#include <Eigen/Geometry>

//parameters for tweeking
float upper_limit= 0.1;
float voxel_size=0.01;

//typing
float a;
float b;
float c;
float d;

//can be set to true if you want to skip calibration!
bool rdy=false;
 
//publishers
ros::Publisher pub;

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
voxel_grid.setLeafSize (voxel_size, voxel_size, voxel_size);
voxel_grid.filter (*inital_cloud);

// Rotation (and a little bit of translation)
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
Eigen::Affine3f transform = Eigen::Affine3f::Identity();
transform.rotate(Eigen::AngleAxisf(-acos(abs(b)), Eigen::Vector3f::UnitX()));
transform.translation() << 0.0f, -d, 0.0f;
pcl::transformPointCloud(*inital_cloud, *transformed_cloud, transform);

//// Filter object for objects (is a filter in height removes NaN) and publish. 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloudObjects (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PassThrough<pcl::PointXYZRGB> pass2;
pass2.setInputCloud (transformed_cloud);
pass2.setFilterFieldName ("y");
pass2.setFilterLimits (-upper_limit,-0.01f);
pass2.filter (*filteredCloudObjects);
sensor_msgs::PointCloud2 cloud_out_objects;
pcl::toROSMsg (*filteredCloudObjects, cloud_out_objects);
pub.publish (cloud_out_objects);
}

//Wait for calibration, when this message is received we wil start generation of the clouds
void run (const std_msgs::Bool& run)
{

//set so that we will start process data in the other callback
rdy=true;

//get calibration data
std::ifstream indata;
indata.open("src/nord/nord_pointcloud/data/calibration.txt");
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

  // Spin
  while(ros::ok())
  {
    ros::spinOnce();
  }
}
