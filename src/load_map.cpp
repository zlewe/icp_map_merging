#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <vector>
#include <cmath>
#include <string>
#include <math.h>
#include <signal.h>
#include <Eigen/Geometry>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>

#include <icp_map_merging/cbshot.h>

std::string PATH = "/home/argsubt/ros_docker_ws/localization_comp/SDC Localization Competition Data/map/nuscenes_map.pcd";

int main(int argc, char** argv){
    ros::init(argc, argv, "load_map");

    ros::NodeHandle nh;
    ros::Publisher map_filtered_pub;
    pcl::PointCloud<pcl::PointXYZI>::Ptr icp_map;
    pcl::PointCloud<pcl::PointXYZI>::Ptr icp_map_voxel;

    sensor_msgs::PointCloud2 ros_cloud_msg;

    pcl::VoxelGrid<pcl::PointXYZI> sor;

    std::string filePath;
    std::string fileName;
    double leafSize;
    nh.getParam("/map_node/file_path", filePath);
    nh.getParam("/map_node/file_name", fileName);
    nh.getParam("/map_node/leaf_size", leafSize);

    PATH = filePath+"/"+fileName;

    map_filtered_pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_filtered", 1, true);

    icp_map.reset(new pcl::PointCloud<pcl::PointXYZI>());
    icp_map_voxel.reset(new pcl::PointCloud<pcl::PointXYZI>());

    std::cerr << "Loading Map " << PATH << std::endl;
    pcl::io::loadPCDFile<pcl::PointXYZI>(PATH, *icp_map);
    std::cerr << "Loaded" << std::endl;

    std::cerr << "PointCloud before filtering: " << icp_map->width * icp_map->height 
        << " data points (" << pcl::getFieldsList (*icp_map) << ")." << std::endl;
    sor.setInputCloud(icp_map);
    sor.setLeafSize(leafSize, leafSize, leafSize);
    sor.filter(*icp_map_voxel);
    std::cerr << "PointCloud after filtering: " << icp_map_voxel->width * icp_map_voxel->height 
        << " data points (" << pcl::getFieldsList (*icp_map_voxel) << ")." << std::endl;

    toROSMsg(*icp_map_voxel, ros_cloud_msg);
    ros_cloud_msg.header.frame_id = "world";
    map_filtered_pub.publish(ros_cloud_msg);

    std::cerr << "Published" << std::endl;

    ros::spin();
}