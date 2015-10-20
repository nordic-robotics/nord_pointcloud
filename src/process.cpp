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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
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
ros::Publisher pub2;

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
sor.setLeafSize (0.01, 0.01, 0.01);
sor.filter (cloud_filtered);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr not_transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
pcl::fromPCLPointCloud2(cloud_filtered, *not_transformed_cloud);


Eigen::Affine3f transform = Eigen::Affine3f::Identity();
transform.rotate(AngleAxisf(-acos(abs(b)), Vector3f::UnitX()));
transform.translation() << 0.0, -d, 0.0;

// Executing the transformation
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
pcl::transformPointCloud(*not_transformed_cloud, *transformed_cloud, transform);


// Filter object.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PassThrough<pcl::PointXYZRGB> pass;
pass.setInputCloud (transformed_cloud);
pass.setFilterFieldName ("y");
pass.setFilterLimits (-0.3,-0.1);
//pass.setFilterLimitsNegative (true);
pass.filter (*filteredCloud);

// Filter object.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PassThrough<pcl::PointXYZRGB> pass2;
pass2.setInputCloud (transformed_cloud);
pass2.setFilterFieldName ("y");
pass2.setFilterLimits (-0.1,-0.01);
//pass.setFilterLimitsNegative (true);
pass2.filter (*filteredCloud2);

// build the condition
//pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
//range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
//pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::GT, 0.01)));
//range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
//   pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::LT, 0.30)));

// build the filter
//pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (range_cond);
//condrem.setInputCloud (transformed_cloud);
//condrem.setKeepOrganized(true);

// apply filter
//condrem.filter (*filteredCloud);



sensor_msgs::PointCloud2 output;
pcl::toROSMsg (*filteredCloud, output);

//to sensor_msgs
sensor_msgs::PointCloud2 output2;
pcl::toROSMsg (*filteredCloud2, output2);

// Publish the data.
pub2.publish (output2);


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
  ros::Subscriber sub=nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);


  ifstream indata;
  indata.open("src/nord/nord_pointcloud/data/calibration.txt");
  indata >> a >> b >> c >> d;

  indata.close();

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub2 = nh.subscribe ("/nord/pointcloud/calibration", 1, run);

  // Create a ROS publisher for the output point cloud
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/nord/pointcloud/processed_withoutwalls", 1);
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/nord/pointcloud/processed_walls", 1);

  // Spin
  ros::spin ();
}
