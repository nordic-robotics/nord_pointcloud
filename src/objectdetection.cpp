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


#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/point_representation.h>
//tweekable parameters
  //debug 

int lowerlimitcluster=20;
int upperlimitcluster=100;
float lowerlimitserach=0.4f;
float upperlimitserach=0.8f;
float ClusterTolerance=0.04f;
ros::Publisher marker_pub;
ros::Publisher centroid_pub;
//ros::Publisher img_pub;

typedef pcl::PointXYZ pointtype;


float a;
float b;
float c;
float d;

//can be set to true if you want to skip calibration!
bool rdy=true;
bool debug=true;

std::vector<float> vhf(const pcl::PointCloud<pointtype>::Ptr object){
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::VFHSignature308> descriptor;
 
  // Estimate the normals.
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(object);
  normalEstimation.setRadiusSearch(0.03);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.compute(*normals);
 
  // VFH estimation object.
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud(object);
  vfh.setInputNormals(normals);
  vfh.setSearchMethod(kdtree);
  // Optionally, we can normalize the bins of the resulting histogram,
  // using the total number of points.
  vfh.setNormalizeBins(true);
  // Also, we can normalize the SDC with the maximum size found between
  // the centroid and any of the cluster's points.
  vfh.setNormalizeDistance(true);
  vfh.compute(descriptor);
  //std::cout << descriptor.size() << std::endl;
  std::vector<float> arr(descriptor.points[0].histogram, descriptor.points[0].histogram + 308);
  return arr;
}

void debugmakers(const nord_messages::CoordinateArray& message){
  //generate a maker for each cluter(object or debris)
  visualization_msgs::Marker marker;
  marker.header.frame_id ="camera_depth_frame";
  marker.id=1;
  marker.ns="detected objects";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::CUBE_LIST;   
  marker.pose.orientation.w = 1.0f;
  marker.scale.x = 0.02f;
  marker.scale.y = 0.02f;
  marker.scale.z = 0.02f;
  marker.color.r = 1.0f;//centroid(3);
  marker.color.g = 0;//centroid(4);
  marker.color.b = 0;//centroid(5);
  marker.color.a = 1.0f;
  marker.lifetime = ros::Duration();
  marker.action = visualization_msgs::Marker::ADD;

  //debris-hull-maker (every object or debris)
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

  for(uint i=0;i <message.data.size();i++){
    geometry_msgs::Point onemarker;
    onemarker.x =message.data[i].x-0.01f;
    onemarker.y =message.data[i].y-0.01f;
    onemarker.z =message.data[i].z-0.01f;
    marker.points.push_back(onemarker);
    for (uint j=0; j<message.data[i].hull.size();j++){
      geometry_msgs::Point twomarker;
      twomarker.x =message.data[i].hull[j].x;
      twomarker.y =message.data[i].hull[j].y;
      twomarker.z =0.0;
      debris.points.push_back(twomarker);
      if (!(j ==0 || j == (message.data[i].hull.size()-1))){
         debris.points.push_back(twomarker);
      }
    }
  }
  marker_pub.publish(marker);
  marker_pub.publish(debris);
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  //wait for calibration. 
  if (!rdy){
      return;
  }

  //Recive timestamp
  ros::Time timestamp= cloud_msg->header.stamp;

  //Recived message
  pcl::PointCloud<pointtype>::Ptr not_transformed_cloud (new pcl::PointCloud<pointtype>());;
  pcl::fromROSMsg (*cloud_msg,*not_transformed_cloud);

  //filter in depth will also remove spurious NaNs
  pcl::PointCloud<pointtype>::Ptr filteredCloudObjectsDepth(new pcl::PointCloud<pointtype>());
  pcl::PassThrough<pointtype> pass;
  pass.setInputCloud (not_transformed_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (lowerlimitserach,upperlimitserach);
  pass.filter (*filteredCloudObjectsDepth);

  // //publish!
  // sensor_msgs::PointCloud2 cloud_out_objects;
  // cloud_out_objects.header.stamp=timestamp;
  // pcl::toROSMsg (*filteredCloudObjectsDepth, cloud_out_objects);
  // img_pub.publish(cloud_out_objects);
  
  //clustering
  pcl::search::KdTree<pointtype>::Ptr tree (new pcl::search::KdTree<pointtype>());
  tree->setInputCloud (filteredCloudObjectsDepth);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pointtype> ec;
  ec.setClusterTolerance (ClusterTolerance);
  ec.setMinClusterSize (lowerlimitcluster);
  ec.setMaxClusterSize (upperlimitcluster);
  ec.setSearchMethod (tree);
  ec.setInputCloud (filteredCloudObjectsDepth);
  ec.extract (cluster_indices);


  uint j = 0;
  nord_messages::CoordinateArray message;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    //for each cluster create a cloud and compute a centroid
    pcl::PointCloud<pointtype>::Ptr cloud_cluster(new pcl::PointCloud<pointtype>());
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back(filteredCloudObjectsDepth->points[*pit]);

    
    //coordinate message
    Eigen::VectorXd centroid;    
    pcl::computeNDCentroid(*cloud_cluster, centroid);
    geometry_msgs::Vector3 vec;
    vec.x=centroid(1);
    vec.y=-centroid(0);
    vec.z = -centroid(2);

    //prioject particles to ground plane
    for (uint p=0;p<cloud_cluster->points.size();p++){
      cloud_cluster->points[p].y=0;
    }
    //for convex hull
    pcl::PointCloud<pointtype> cloud_hull;
    pcl::ConvexHull<pointtype> chull;
    chull.setInputCloud (cloud_cluster);
    chull.reconstruct (cloud_hull);


    nord_messages::Vector2 vec0;
    nord_messages::Coordinate add;
    for(uint i=0; i<cloud_hull.points.size(); i++){
      vec0.x=cloud_hull.points[i].z;
      vec0.y=-cloud_hull.points[i].x;
      add.hull.push_back(vec0);    
    }


    //matrix for camera
    static Eigen::Matrix<float,3,4> P;
    P << 574.0527954101562, 0.0, 319.5, 0.0,
    0.0, 574.0527954101562, 239.5, 0.0,
    0.0, 0.0, 1.0, 0.0;  

    //rotate back to image coorindates
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.0f, d, 0.0f;
    transform.rotate(Eigen::AngleAxisf(acos(abs(b)), Eigen::Vector3f::UnitX()));
    Eigen::Vector3f in= transform*Eigen::Vector3f(centroid(0),centroid(1), centroid(2));
    Eigen::Vector3f imagecoords = P * Eigen::Vector4f(in(0),in(1),in(2), 1);
    imagecoords=imagecoords/imagecoords[2];

    //populate message 
    add.x=centroid(2);
    add.y=-centroid(0);
    add.z=-centroid(1);
    add.xp=imagecoords(0);
    add.yp=imagecoords(1);
    add.VFH=vhf(cloud_cluster);
    message.data.push_back(add);
    message.stamp=timestamp;

    //next object in loop
    j++;
  }
  if (debug){
    debugmakers(message);
  }
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

  std::cout << "Calibration received" << std::endl;
}

int main (int argc, char** argv){
  // Initialize ROS and cretate a handle
  ros::init (argc, argv, "objectdetection");
  ros::NodeHandle nh;

  //subscribers
  ros::Subscriber sub =nh.subscribe ("/nord/pointcloud/processed", 1, cloud_cb);
  ros::Subscriber sub2 = nh.subscribe ("/nord/pointcloud/calibration", 1, run);

  //publishers
  marker_pub = nh.advertise<visualization_msgs::Marker>("/nord/pointcloud/visualization_marker", 1);
  centroid_pub = nh.advertise<nord_messages::CoordinateArray>("/nord/pointcloud/centroids",1);
  //img_pub = nh.advertise<sensor_msgs::PointCloud2>("/nord/pointcloud/depth",1);


  ros::spin();
}