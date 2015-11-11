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
#include <sstream>
#include <fstream>


//tweaking!!!!
uint a=2; // amount of consideration to varraince in y or x (depending on direction)
uint b=0.01; // amount of fixed size (width of wall in cm. OBS! this half of width )
uint c=1; //amount of consideration of variance in theta. (0 now, may be useful later...tweak it?)

ros::Publisher no_wall_pub;
nord_messages::PoseEstimate pose;

typedef pcl::PointXYZ pointtype;

struct line{
line(float x0, float x1, float y0, float y1)
: x0(x0), x1(x1), y0(y0), y1(y1){}
float x0;
float x1;
float y0;
float y1;
};

std::vector<line> maze;


std::vector<line> read_map(std::string filename){
    std::ifstream file(filename.c_str());
    std::string l;
    std::vector<line> walls;
    while (std::getline(file, l))
    {
        std::istringstream iss(l);
        if (l[0] == '#')
            continue;

        float x0, y0, x1, y1;
        iss >> x0 >> y0 >> x1 >> y1;
        walls.push_back(line(x0, x1, y0, y1));
    }
    return walls;
}

pcl::ConditionOr<pcl::PointXYZ>::Ptr conditionbox(line linen){
    // Checks available: GT, GE, LT, LE, EQ.
pcl::ConditionOr<pcl::PointXYZ>::Ptr condition(new pcl::ConditionOr<pcl::PointXYZ>());
float xmin = std::min(linen.x0,linen.x1)-pose.x.stddev*a-b-pose.x.mean-pose.theta.stddev*c;
float xmax = std::max(linen.x0,linen.x1)+pose.x.stddev*a+b-pose.x.mean+pose.theta.stddev*c;
float ymin = std::min(-linen.y0,-linen.y1)-pose.y.stddev*a-b-pose.y.mean-pose.theta.stddev*c;
float ymax = std::max(-linen.y0,-linen.y1)+pose.y.stddev*a+b-pose.y.mean+pose.theta.stddev*c;
condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, xmin)));
condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, xmax)));
condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, ymin)));
condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, ymax)));
//std::cout << xmin << " " << xmax << " " << ymin << " " << ymax << std::endl;
 return condition;
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

  //allign cloud with theta
  pcl::PointCloud<pointtype>::Ptr transformed_cloud(new pcl::PointCloud<pointtype>());
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.rotate(Eigen::AngleAxisf(-pose.theta.mean, Eigen::Vector3f::UnitY()));
  pcl::transformPointCloud(not_transformed_cloud, *transformed_cloud, transform);

  pcl::ConditionAnd<pointtype>::Ptr condition(new pcl::ConditionAnd<pointtype>());
  for (uint j=0;j<maze.size();j++){
    condition->addCondition(conditionbox(maze[j]));
  }

  // Filter object.
  pcl::PointCloud<pointtype>filteredCloud;
  pcl::ConditionalRemoval<pointtype> filter;
  filter.setCondition(condition);
  filter.setInputCloud(transformed_cloud);
  filter.setKeepOrganized(false);
  filter.filter(filteredCloud);

// std::cout << transformed_cloud->points.size() << std::endl;
// std::cout << filteredCloud.points.size() << std::endl;

  //allign cloud with theta
  pcl::PointCloud<pointtype> return_cloud;
  Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
  transform2.rotate(Eigen::AngleAxisf(pose.theta.mean, Eigen::Vector3f::UnitY()));
  pcl::transformPointCloud(filteredCloud, return_cloud, transform2);


  //publish!
  sensor_msgs::PointCloud2 cloud_out_objects;
  cloud_out_objects.header.stamp=timestamp;
  pcl::toROSMsg (return_cloud, cloud_out_objects);
  no_wall_pub.publish (cloud_out_objects);
  
}

int main (int argc, char** argv){
  // Initialize ROS and cretate a handle
  ros::init (argc, argv, "cleverwallremoval");
  ros::NodeHandle nh;

  //saker som händer en gång i min main
  maze = read_map(ros::package::getPath("nord_pointcloud") + "/data/small_maze.txt");

  //subscribers
  ros::Subscriber sub =nh.subscribe ("/nord/pointcloud/processed", 1, cloud_cb);
  ros::Subscriber guess_pub = nh.subscribe("/nord/estimation/gaussian", 1, filter_cb);

  //publishers
  no_wall_pub = nh.advertise<sensor_msgs::PointCloud2>("/nord/pointcloud/no_wall", 1);



  ros::spin();
}