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
#include <nord_messages/Coordinate.h>
#include <nord_messages/CoordinateArray.h>


//using namespace
using namespace Eigen;
using namespace std;

ros::Publisher marker_pub;
ros::Publisher centroid_pub;
uint32_t shape = visualization_msgs::Marker::CUBE;

float a;
float b;
float c;
float d;


//can be set to true if you want to skip calibration!
bool rdy=true;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
//wait for calibration. 
if (!rdy){
    return;
}
//Recived message
pcl::PointCloud<pcl::PointXYZRGB>::Ptr not_transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
pcl::fromROSMsg (*cloud_msg,*not_transformed_cloud);

//filter in depth
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloudObjectsDepth (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PassThrough<pcl::PointXYZRGB> pass3;
pass3.setInputCloud (not_transformed_cloud);
pass3.setFilterFieldName ("z");
pass3.setFilterLimits (0.4f,0.8f);
pass3.filter (*filteredCloudObjectsDepth);

pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
tree->setInputCloud (filteredCloudObjectsDepth);
std::vector<pcl::PointIndices> cluster_indices;
pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
ec.setClusterTolerance (0.02); // 2cm
ec.setMinClusterSize (10);
ec.setMaxClusterSize (50);
ec.setSearchMethod (tree);
ec.setInputCloud (filteredCloudObjectsDepth);
ec.extract (cluster_indices);

static Eigen::Matrix<float,3,4> P;
P << 574.0527954101562, 0.0, 319.5, 0.0,
     0.0, 574.0527954101562, 239.5, 0.0,
     0.0, 0.0, 1.0, 0.0;


int j = 0;
nord_messages::CoordinateArray message;
for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
{
    //for each cluster create a cloud and compute a centroid
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (filteredCloudObjectsDepth->points[*pit]); //*
    Eigen::VectorXd centroid;    
    pcl::computeNDCentroid(*cloud_cluster, centroid);

    //generate a maker for rviz for each cloud cluster of kmeans
    visualization_msgs::Marker marker;
    marker.header.frame_id ="camera_depth_frame";
    marker.id=j;
    marker.ns="detected objects";
    marker.header.stamp = ros::Time::now();
    marker.type = shape;
    marker.pose.position.x = centroid(2);
    marker.pose.position.y = -centroid(0);
    marker.pose.position.z = -centroid(1);
    marker.pose.orientation.x = 0.0f;
    marker.pose.orientation.y = 0.0f;
    marker.pose.orientation.z = 0.0f;
    marker.pose.orientation.w = 1.0f;
    marker.scale.x = 0.05f;
    marker.scale.y = 0.05f;
    marker.scale.z = 0.05f;
    marker.color.r = 1.0f;//centroid(3);
    marker.color.g = 0;//centroid(4);
    marker.color.b = 0;//centroid(5);
    marker.color.a = 1.0f;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);
    geometry_msgs::Vector3 vec;
    vec.x=centroid(2);
    vec.y=-centroid(0);
    vec.z = -centroid(1);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.0f, d, 0.0f;
    transform.rotate(AngleAxisf(acos(abs(b)), Vector3f::UnitX()));
    Eigen::Vector3f in= transform*Eigen::Vector3f(centroid(0),centroid(1), centroid(2));
    Eigen::Vector3f imagecoords = P * Eigen::Vector4f(in(0),in(1),in(2), 1);
    imagecoords=imagecoords/imagecoords[2];
    //std::cout << imagecoords[0] << "," << imagecoords[1] << endl;

    nord_messages::Coordinate add;
    add.x=centroid(0);
    add.y=centroid(1);
    add.z=centroid(2);
    add.xp=imagecoords(0);
    add.yp=imagecoords(1);
    message.data.push_back(add);


    //next object in loop
    j++;

}

centroid_pub.publish(message);
}

//Wait for calibration, when this message is received we wil start generation of the clouds
void run (const std_msgs::Bool& run)
{
rdy=true;
}

int main (int argc, char** argv){

  // Initialize ROS and cretate a handle
  ros::init (argc, argv, "objectdetection");
  ros::NodeHandle nh;

  //subscribers
  ros::Subscriber sub=nh.subscribe ("/nord/pointcloud/processed_objectdebris", 1, cloud_cb);
  ros::Subscriber sub2 = nh.subscribe ("/nord/pointcloud/calibration", 1, run);

  //get calibration data
  ifstream indata;
  indata.open("src/nord/nord_pointcloud/data/calibration.txt");
  indata >> a >> b >> c >> d;
  indata.close();

  //publishers
  marker_pub = nh.advertise<visualization_msgs::Marker>("/nord/pointcloud/visualization_marker", 1);
  centroid_pub = nh.advertise<nord_messages::CoordinateArray>("/nord/pointcloud/centroids",1);

  // Spin
  while(ros::ok())
  {
    ros::spinOnce();
  }
}