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
#include <pcl/common/centroid.h>
#include <nord_messages/Coordinate.h>
#include <nord_messages/CoordinateArray.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>



ros::Publisher marker_pub;
ros::Publisher centroid_pub;

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

//filter in depth will also  remove spurious NaNs
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

    //generate a maker 
    visualization_msgs::Marker marker;
    marker.header.frame_id ="camera_depth_frame";
    marker.id=1;
    marker.ns="detected objects";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CUBE_LIST;   
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

        //generate a maker 
    visualization_msgs::Marker debris;
    debris.header.frame_id ="camera_depth_frame";
    debris.id=2;
    debris.ns="detected debris";
    debris.header.stamp = ros::Time::now();
    debris.type = visualization_msgs::Marker::LINE_LIST;
    debris.scale.x = 0.01f;    
    marker.pose.orientation.w = 1.0f;
    debris.color.r = 0;//centroid(3);
    debris.color.g = 0;//centroid(4);
    debris.color.b = 1.0f;//centroid(5);
    debris.color.a = 1.0f;
    debris.lifetime = ros::Duration();
    debris.action = visualization_msgs::Marker::ADD;



  // Create a set of planar coefficients with X=Z=0,Y=-1 height=0 
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[2] = 0;
  coefficients->values[1] = -1.0;
  coefficients->values[3] = 0;


uint j = 0;
nord_messages::CoordinateArray message;
for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
{
  //for each cluster create a cloud and compute a centroid
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    cloud_cluster->points.push_back (filteredCloudObjectsDepth->points[*pit]); //*

//prioject particles to ground plane for convex hull
  for (uint p=0;p<cloud_cluster->points.size();p++){
    cloud_cluster->points[p].y=0;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ConvexHull<pcl::PointXYZRGB> chull;
  chull.setInputCloud (cloud_cluster);
  chull.reconstruct (*cloud_hull);

  uint i=0;
  geometry_msgs::Point twomarker;
  nord_messages::Coordinate add;
  nord_messages::Vector2 vec0;

  while (i< cloud_hull->points.size()){
    vec0.x=cloud_hull->points[i].z;
    vec0.y=-cloud_hull->points[i].x;
    add.hull.push_back(vec0);
    twomarker.y=-cloud_hull->points[i].x;
    twomarker.x=cloud_hull->points[i].z;
    twomarker.z=0;
    debris.points.push_back(twomarker);
    if (!(i ==0 || i == (cloud_hull->points.size()-1))){
      debris.points.push_back(twomarker);
    }
    i++;
  }
  twomarker.y=-cloud_hull->points[cloud_hull->points.size()-1].x;
  twomarker.x=cloud_hull->points[cloud_hull->points.size()-1].z;
  debris.points.push_back(twomarker);

  twomarker.y=-cloud_hull->points[0].x;
  twomarker.x=cloud_hull->points[0].z;
  debris.points.push_back(twomarker);

  //marker stuff!!!
  Eigen::VectorXd centroid;    
  pcl::computeNDCentroid(*cloud_cluster, centroid);
  geometry_msgs::Point onemarker;
  onemarker.x = centroid(2);
  onemarker.y = -centroid(0);
  onemarker.z = -centroid(1);
  marker.points.push_back(onemarker);

  //centroid message stuff here below!!!
  geometry_msgs::Vector3 vec;
  vec.x=centroid(1);
  vec.y=-centroid(0);
  vec.z = -centroid(2);

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << 0.0f, d, 0.0f;
  transform.rotate(Eigen::AngleAxisf(acos(abs(b)), Eigen::Vector3f::UnitX()));
  Eigen::Vector3f in= transform*Eigen::Vector3f(centroid(0),centroid(1), centroid(2));
  Eigen::Vector3f imagecoords = P * Eigen::Vector4f(in(0),in(1),in(2), 1);
  imagecoords=imagecoords/imagecoords[2];

  add.x=centroid(2);
  add.y=-centroid(0);
  add.z=-centroid(1);
  add.xp=imagecoords(0);
  add.yp=imagecoords(1);
  message.data.push_back(add);

  //next object in loop
  j++;
}

marker_pub.publish(marker);
marker_pub.publish(debris);
centroid_pub.publish(message);
}

//Wait for calibration, when this message is received we wil start generation of the clouds
void run (const std_msgs::Bool& run)
{
  rdy=true;

  //get calibration data
  std::ifstream indata;
  indata.open("src/nord/nord_pointcloud/data/calibration.txt");
  indata >> a >> b >> c >> d;
  indata.close();

  std::cout << "Calibration received";
}

int main (int argc, char** argv){
  // Initialize ROS and cretate a handle
  ros::init (argc, argv, "objectdetection");
  ros::NodeHandle nh;

  //subscribers
  ros::Subscriber sub =nh.subscribe ("/nord/pointcloud/processed_objectdebris", 1, cloud_cb);
  ros::Subscriber sub2 = nh.subscribe ("/nord/pointcloud/calibration", 1, run);

  //publishers
  marker_pub = nh.advertise<visualization_msgs::Marker>("/nord/pointcloud/visualization_marker", 1);
  centroid_pub = nh.advertise<nord_messages::CoordinateArray>("/nord/pointcloud/centroids",1);


  // Spin
  while(ros::ok())
  {
    ros::spinOnce();
  }
}