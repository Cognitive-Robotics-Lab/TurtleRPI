//From http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation

#include <ros/ros.h>
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
#include <TurtleKinect/FloorConfig.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

//publishers
ros::Publisher cloud_pub;

//params
int   ModelType; 
int   MaxIterations;
float DistanceThreshold;
float RatioLimit;
bool ExtractNegative;


// enum SacModel
//  {
//  SACMODEL_PLANE,
//  SACMODEL_LINE,
//  SACMODEL_CIRCLE2D,
//  SACMODEL_CIRCLE3D,
//  SACMODEL_SPHERE,
//  SACMODEL_CYLINDER,
//  SACMODEL_CONE,
//  SACMODEL_TORUS,
//  SACMODEL_PARALLEL_LINE,
//  SACMODEL_PERPENDICULAR_PLANE,
//  SACMODEL_PARALLEL_LINES,
//  SACMODEL_NORMAL_PLANE,
//  SACMODEL_NORMAL_SPHERE,
//  SACMODEL_REGISTRATION,
//  SACMODEL_PARALLEL_PLANE,
//  SACMODEL_NORMAL_PARALLEL_PLANE,
//  SACMODEL_STICK
//  };

void cloud_cb (sensor_msgs::PointCloud2ConstPtr input) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg (*input, *cloud_filtered);
  //std::cout << "Input pointCloud has: " << cloud_filtered->points.size () << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (ModelType);//(pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (MaxIterations);//(100);
  seg.setDistanceThreshold (DistanceThreshold);//(0.02);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > RatioLimit * nr_points)//0.3 * nr_points)
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
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    // extract.setNegative (false);

    // // Write the planar inliers to disk
    // extract.filter (*cloud_plane);
    // std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (ExtractNegative);
    extract.filter (*cloud_f);
    cloud_filtered = cloud_f;
  }


  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg (*cloud_filtered , output);
  output.header.stamp = ros::Time::now();
  output.header.frame_id = "/camera_rgb_optical_frame";
  // Publish the data
  cloud_pub.publish (output);
}

void config_callback(TurtleKinect::FloorConfig &config, uint32_t level) {
  ModelType = config.ModelType;
  MaxIterations = config.MaxIterations;
  DistanceThreshold = config.DistanceThreshold;
  RatioLimit = config.RatioLimit;
  ExtractNegative = config.ExtractNegative;
}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "floor_segment");
  ros::NodeHandle nh("~");

  dynamic_reconfigure::Server<TurtleKinect::FloorConfig> server;
  dynamic_reconfigure::Server<TurtleKinect::FloorConfig>::CallbackType f = boost::bind(&config_callback, _1, _2);
  server.setCallback(f);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);


  // Spin
  ros::spin ();
}