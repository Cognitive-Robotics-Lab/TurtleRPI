#include <ros/ros.h>
//From http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction

#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/impl/centroid.hpp>
#include <stdlib.h>
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include <TurtleKinect/ClusterConfig.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

//publishers
ros::Publisher pose_pub;
ros::Publisher cloud_pub;

//params
float ClusterTolerance;
int   MinClusterSize;
int   MaxClusterSize;

void cloud_cb (sensor_msgs::PointCloud2ConstPtr input) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud_filtered);
  std::cout << "Input pointCloud has: " << cloud_filtered->points.size () << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  /*pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Write the planar inliers to disk
    extract.filter (*cloud_plane);
    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered = cloud_f;
  }*/

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (ClusterTolerance); // 2cm
  ec.setMinClusterSize (MinClusterSize);
  ec.setMaxClusterSize (MaxClusterSize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<Eigen::Vector4f> centroids;

  std::cout << "\t\t" << cluster_indices.size() << "Clusters:" ;
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    uint8_t r = rand()%255, g = rand()%255, b = rand()%255;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {
      pcl::PointXYZRGB p(r,g,b);
      p.x = cloud_filtered->points[*pit].x;
      p.y = cloud_filtered->points[*pit].y;
      p.z = cloud_filtered->points[*pit].z;
      cloud_cluster->points.push_back (p); //*
    }
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << cloud_cluster->points.size () ;
    stringstream ss; ss << ":(" << (int)r << "," << (int)g << ","<< (int)b << ")";
    std::cout << ss.str() << " , ";

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster,centroid);
    centroids.push_back(centroid);
    *cloud_clustered += *cloud_cluster;
    j++;
  }
  std::cout << std::endl;

  geometry_msgs::PoseArray poses;
  poses.header.stamp = ros::Time::now();
  poses.header.frame_id = "/camera_rgb_optical_frame";
  for (std::vector<Eigen::Vector4f>::const_iterator it = centroids.begin (); it != centroids.end (); ++it) {
    geometry_msgs::Pose pose;
    pose.position.x = it->x();
    pose.position.y = it->y();
    pose.position.z = it->z();
    poses.poses.push_back(pose);
  }
  pose_pub.publish (poses);


  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg (*cloud_clustered , output);
  output.header.stamp = ros::Time::now();
  output.header.frame_id = "/camera_rgb_optical_frame";
  // Publish the data
  cloud_pub.publish (output);
}

void config_callback(TurtleKinect::ClusterConfig &config, uint32_t level) {
  ClusterTolerance = config.ClusterTolerance;
  MinClusterSize = config.MinClusterSize;
  MaxClusterSize = config.MaxClusterSize;
}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "turtle_cluster");
  ros::NodeHandle nh("~");

  dynamic_reconfigure::Server<TurtleKinect::ClusterConfig> server;
  dynamic_reconfigure::Server<TurtleKinect::ClusterConfig>::CallbackType f = boost::bind(&config_callback, _1, _2);
  server.setCallback(f);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);


  pose_pub = nh.advertise<geometry_msgs::PoseArray> ("centroids", 1);

  // Spin
  ros::spin ();
}