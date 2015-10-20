#include <ros/ros.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <fstream>
#include <valarray>
#include <vector>

using namespace std;
std::vector <valarray<float> >acc;
ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::fromROSMsg (*input, cloud);

//remove upper half of cloud? not very useful + saves computations.

pcl::ModelCoefficients coefficients;
pcl::PointIndices inliers;
// Create the segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg;
// Optional
seg.setOptimizeCoefficients (true);
// Mandatory
seg.setModelType (pcl::SACMODEL_PLANE);
seg.setMethodType (pcl::SAC_RANSAC);
seg.setDistanceThreshold (0.01);
seg.setAxis(Eigen::Vector3f(0,1,0));

seg.setInputCloud (cloud.makeShared ());
seg.segment (inliers, coefficients);
if (coefficients.values[0]==coefficients.values[0]){
acc.push_back(valarray<float> (coefficients.values.data(),coefficients.values.size()));
}
// Publish the model coefficients
pcl_msgs::ModelCoefficients ros_coefficients;
pcl_conversions::fromPCL(coefficients, ros_coefficients);

}
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "calibration");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<std_msgs::Bool> ("/nord/pointcloud/calibration", 1);

  while (ros::ok() && acc.size()<20){
      ros::spinOnce();

  }
  std::valarray<float> sum(4);

  for (size_t i=0;i<acc.size();i++){
    sum+=acc[i];
   }

  sum/=acc.size();

  ofstream myfile;
  myfile.open ("src/nord/nord_pointcloud/data/calibration.txt");
  for (size_t i=0;i<sum.size();i++){
  myfile << sum[i] << "\n";
  }

  myfile.close();
  std_msgs::Bool run;
  run.data=true;
  pub.publish(run);
}
