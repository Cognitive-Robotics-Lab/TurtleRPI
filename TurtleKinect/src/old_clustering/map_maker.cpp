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
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/impl/centroid.hpp>
#include <stdlib.h>
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include <TurtleKinect/MapConfig.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/Image.h>
#include <pcl/filters/passthrough.h>
//publishers
ros::Publisher cloud_pub;
ros::Publisher image_pub;
float DistanceThreshold;
int   MaxIterations;

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

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                          cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                          cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg (*input, *cloud);
  std::cout << "Input pointCloud has: " << cloud->points.size () << " data points." << "\n"; 

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-5.0, 5.0);
  pass.setUserFilterValue (0);
  pass.setKeepOrganized(true);
  pass.filter (*cloud_filtered);

  std::cout << "PassThrough pointCloud has: " << cloud_filtered->points.size () << " data points." << "\n"; 
  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (DistanceThreshold);
  seg.setMaxIterations (MaxIterations);//(100);
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << "\n";

  // Extract the planar inliers from the input cloud
  //.pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  // extract.setInputCloud (cloud_filtered);
  // extract.setIndices (inliers);
  // extract.setNegative (false);
  // extract.filter (*cloud_f);


  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  std::cerr << "PointCloud after projection has: "
            << cloud_projected->points.size () << " data points." << "\n";

  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ConcaveHull<pcl::PointXYZRGB> chull;
  chull.setInputCloud (cloud_projected);
  chull.setAlpha (0.2);
  chull.setKeepInformation (false);
  chull.reconstruct (*cloud_hull);

  if(cloud_hull->points.size() < 9000) {
    std::cerr << "Concave hull has: " << cloud_hull->points.size ()
              << " data points." << "\n";

    std::cerr << "Creating output map cloud.\n\n";


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud_projected->width = cloud->width;
    cloud_projected->height = cloud->height;
    cloud_out->width  = cloud_projected->width;
    cloud_out->height  = cloud_projected->height;
    cloud_out->points.resize( cloud_projected->width * cloud_projected->height);
    pcl::PointXYZRGB hull_point(0,0,0);
    for (size_t y = 0; y < cloud_out->height; y++) {
      for (size_t x = 0; x < cloud_out->width; x++) {
        pcl::PointXYZRGB cp(255,255,255);
        int index = x+cloud_out->width*y;
        pcl::PointXYZRGB* pp = &cloud_projected->points[index];
        cp.x = pp->x;
        cp.y = pp->y;
        cp.z = pp->z; 

        for (std::vector<pcl::PointXYZRGB,Eigen::aligned_allocator<pcl::PointXYZRGB> >::iterator pit = cloud_hull->points.begin (); pit != cloud_hull->points.end (); pit++) {
          if((*pit).x==cp.x && (*pit).y==cp.y && (*pit).z==cp.z) {
            cp.rgb = hull_point.rgb;
            break;
          }          
        }

        cloud_out->points[index] = cp;
      }
    }
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*cloud_hull , output);
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "/camera_rgb_optical_frame";
    // Publish the data
    cloud_pub.publish (output);
    
    sensor_msgs::Image image_;
    try
    {
      pcl::toROSMsg (*cloud_out, image_); //convert the cloud
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: "
                        << e.what());
    }
    image_pub.publish (image_); //publish our cloud image
  }
}

void config_callback(TurtleKinect::MapConfig &config, uint32_t level) {
  DistanceThreshold = config.DistanceThreshold;
  MaxIterations = config.MaxIterations;
}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "map_maker");
  ros::NodeHandle nh("~");

  dynamic_reconfigure::Server<TurtleKinect::MapConfig> server;
  dynamic_reconfigure::Server<TurtleKinect::MapConfig>::CallbackType f = boost::bind(&config_callback, _1, _2);
  server.setCallback(f);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  image_pub = nh.advertise<sensor_msgs::Image> ("map_image", 1);


  // Spin
  ros::spin ();
}