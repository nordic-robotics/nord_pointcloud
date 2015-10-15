#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <std_msgs/Bool.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <Eigen/Geometry>
//using namespace

using namespace Eigen;
using namespace std;


ros::Publisher pub;
float a;
float b;
float c;
float d;
bool rdy;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
if (!rdy){
    return;
}
//Container for original & filtered data
pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2();


pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
pcl::PCLPointCloud2 cloud_filtered;
// Convert to PCL data type
pcl_conversions::toPCL(*cloud_msg, *cloud);
// Perform the actual filtering
pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
sor.setInputCloud (cloudPtr);
sor.setLeafSize (0.1, 0.1, 0.1);
sor.filter (cloud_filtered);

pcl::PointCloud<pcl::PointXYZ>::Ptr not_transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::fromPCLPointCloud2(cloud_filtered, *not_transformed_cloud);


Eigen::Affine3f transform = Eigen::Affine3f::Identity();
cout << -acos(abs(b)) << endl;
transform.rotate(AngleAxisf(-acos(abs(b)), Vector3f::UnitX()));
transform.translation() << 0.0, -d, 0.0;

// Executing the transformation
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::transformPointCloud(*not_transformed_cloud, *transformed_cloud, transform);

//to sensor_msgs
sensor_msgs::PointCloud2 output;
pcl::toROSMsg (*transformed_cloud, output);


// Publish the data.
pub.publish (output);
}

void
run (const std_msgs::Bool& run)
{
rdy=true;
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "process");
  ros::NodeHandle nh;
  ros::Subscriber sub=nh.subscribe ("/camera/depth/points", 1, cloud_cb);


  ifstream indata;
  indata.open("src/nord/nord_pointcloud/data/calibration.txt");
  indata >> a >> b >> c >> d;

  indata.close();

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub2 = nh.subscribe ("/nord/pointcloud/calibration", 1, run);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/nord/pointcloud/processed", 1);

  // Spin
  ros::spin ();
}
