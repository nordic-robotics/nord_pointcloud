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
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

//tweaking!!!!
uint a=4; // amount of consideration to varraince in y or x (depending on direction)
uint b=0.02; // amount of fixed size (width of wall in cm. OBS! this half of width )
uint c=0; //amount of consideration of variance in theta. (0 now, may be useful later...tweak it?)
uint remainingspoints=5;

ros::Publisher no_wall_pub;
typedef pcl::PointXYZ pointtype;

struct line{
line(float x0, float x1, float y0, float y1)
: x0(x0), x1(x1), y0(y0), y1(y1){}
float x0;
float x1;
float y0;
float y1;
};


pcl::ConditionOr<pcl::PointXYZ>::Ptr conditionbox(line linen){
    // Checks available: GT, GE, LT, LE, EQ.
pcl::ConditionOr<pcl::PointXYZ>::Ptr condition(new pcl::ConditionOr<pcl::PointXYZ>());
float xmin = std::min(linen.x0,linen.x1)-pose.x.stddev*a-b-pose.x.mean-pose.theta.stddev*c;
float xmax = std::max(linen.x0,linen.x1)+pose.x.stddev*a+b-pose.x.mean+pose.theta.stddev*c;
float ymin = std::min(-linen.y0,-linen.y1)-pose.y.stddev*a-b+pose.y.mean-pose.theta.stddev*c;
float ymax = std::max(-linen.y0,-linen.y1)+pose.y.stddev*a+b+pose.y.mean+pose.theta.stddev*c;
condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, xmin)));
condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, xmax)));
condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, ymin)));
condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, ymax)));
//std::cout << xmin << " " << xmax << " " << ymin << " " << ymax << std::endl;
 return condition;
}


std::vector <std::vector<float> >acc;

void cloud_cb2(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  //clear
  acc.clear();

  //input
  pcl::PointCloud<pointtype>::Ptr wallcloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*cloud_msg, *wallcloud_in); 

  //output-segment
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::PointCloud<pointtype>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

  //segmentation
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.11);
  seg.setMaxIterations(1000);

  //filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  //remove planes and fit the parameters  
  while (wallcloud_in->points.size () > remainingspoints)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (wallcloud_in);
    seg.segment (*inliers, *coefficients);

    //utilize coefficients
    if (coefficients.values[0]==coefficients.values[0]){
    acc.push_back(std::vector<float>(coefficients.values.data(),coefficients.values.size()));
    std::cout << "." <<std::flush;
    }

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    wallcloud_in.swap (cloud_f);
  }


}
void filter_cb(const nord_messages::PoseEstimate::ConstPtr& filterguess){
  pose=*filterguess;
}
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  //Recive timestamp
  ros::Time timestamp= cloud_msg->header.stamp;

  //Recived message
  pcl::PointCloud<pointtype> not_transformed_cloud;
  pcl::fromROSMsg (*cloud_msg, not_transformed_cloud);

  pcl::ConditionAnd<pointtype>::Ptr condition(new pcl::ConditionAnd<pointtype>());
  for (uint j=0;j<acc.size();j++){
    condition->addCondition(conditionbox(acc[j]));
  }

  // Filter object.
  pcl::PointCloud<pointtype>filteredCloud;
  pcl::ConditionalRemoval<pointtype> filter;
  filter.setCondition(condition);
  filter.setInputCloud(transformed_cloud);
  filter.setKeepOrganized(false);
  filter.filter(filteredCloud);

  std::cout << transformed_cloud->points.size() << std::endl;
  std::cout << filteredCloud.points.size() << std::endl;

  //publish!
  sensor_msgs::PointCloud2 cloud_out_objects;
  cloud_out_objects.header.stamp=timestamp;
  pcl::toROSMsg (filteredCloud, cloud_out_objects);
  no_wall_pub.publish (cloud_out_objects);  
}

int main (int argc, char** argv){
  // Initialize ROS and cretate a handle
  ros::init (argc, argv, "evenmorecleverwallremoval");
  ros::NodeHandle nh;

  //subscribers
  ros::Subscriber sub =nh.subscribe ("/nord/pointcloud/processed", 1, cloud_cb);
  ros::Subscriber sub2=nh.subscribe("/nord/pointcloud/walls", 1, cloud_cb2)

  //publishers
  no_wall_pub = nh.advertise<sensor_msgs::PointCloud2>("/nord/pointcloud/no_wall", 1);

  ros::spin();
}