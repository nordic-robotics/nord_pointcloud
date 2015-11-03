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

//tweekable
uint number=10;

//global var
std::vector <std::valarray<float> >acc;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);


//RANSAC fit the floor to the biggest horizontal plane (closest to plan with normals below (0,1,0) attain coefficients and inliers for that plane)
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01f);
  seg.setAxis(Eigen::Vector3f(0,1,0));
  seg.setInputCloud (cloud.makeShared());
//can we avoid saving inliers? Null-pointer?
  seg.segment (inliers, coefficients);

//is not NaN
  if (coefficients.values[0]==coefficients.values[0]){
    acc.push_back(std::valarray<float> (coefficients.values.data(),coefficients.values.size()));
    std::cout << "." <<std::flush;

  }
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "calibration");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output that its rdy
  ros::Publisher pub = nh.advertise<std_msgs::Bool> ("/nord/pointcloud/calibration", 1);

  std::cout << "Calibrating" << std::flush;

  //get 5 mesurements of the plan
  while (ros::ok() && acc.size()<number){
      ros::spinOnce();
  }

  std::cout << std::endl;

  //then average each component 
  std::valarray<float> sum(4);
  for (size_t i=0;i<acc.size();i++){
    sum+=acc[i];
  }
  sum/=acc.size();

  //create the calibration textfile
  std::ofstream myfile;
  myfile.open ("src/nord/nord_pointcloud/data/calibration.txt");
  for (size_t i=0;i<sum.size();i++){
    myfile << sum[i] << "\n";
    //std::cout << sum[i] << endl;
  }
  myfile.close();

  //create message and publish to rdy
  std_msgs::Bool run;
  run.data=true;
  pub.publish(run);

  std::cout << "Done!" << std::endl;

}
