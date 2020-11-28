#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>

using namespace ros;
using namespace std;

class Pointcloud
{
  Publisher pub_map, pub_bag, pub_fin;
  Subscriber sub_bag;
  sensor_msgs::PointCloud2 map_cloud, fin_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;
  Eigen::Matrix4f initial_guess;

public:
  Pointcloud(ros::NodeHandle nh);
  void callback(const sensor_msgs::PointCloud2 &msg);
  
};

Pointcloud::Pointcloud(ros::NodeHandle nh){

  pcl_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();

  initial_guess << -0.866,   -0.5,   0,  -2,
                      0.5, -0.866,   0,   7,
                        0,      0,   1,  -2,
                        0,      0,   0,   1;

  // if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (path + "/map.pcd", *pcl_cloud) == -1) //* load the file 
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/argsubt/ros_docker_ws/hw4/map.pcd", *pcl_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file map.pcd \n");
    exit(0);
  }
  // std::cout << "Loaded "
  //           << pcl_cloud->width * pcl_cloud->height
  //           << " data points from map.pcd with the following fields: "
  //           << std::endl;

//=======voxel grid filter=====================
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor_map;
  pcl::PCLPointCloud2::Ptr map_cloud2 (new pcl::PCLPointCloud2 ());
  pcl::toPCLPointCloud2(*pcl_cloud, *map_cloud2);
  sor_map.setInputCloud (map_cloud2);
  sor_map.setLeafSize (0.5f, 0.5f, 0.5f);
  sor_map.filter (*map_cloud2);
  pcl::fromPCLPointCloud2(*map_cloud2, *pcl_cloud);

  for(int i = 0; i < pcl_cloud->points.size(); i++){
    uint8_t r=0, g=255, b=0;
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    pcl_cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
  }

  pcl::toROSMsg(*pcl_cloud, map_cloud);
  pub_map = nh.advertise<sensor_msgs::PointCloud2>("/map_cloud", 10);
  pub_bag = nh.advertise<sensor_msgs::PointCloud2>("/bag_cloud", 10);
  pub_fin = nh.advertise<sensor_msgs::PointCloud2>("/fin_cloud", 10);

  // sub_bag = nh.subscribe("points_raw", 10, &Pointcloud::callback, this);
  sub_bag = nh.subscribe("/lidar_points", 10, &Pointcloud::callback, this);
}

void Pointcloud::callback(const sensor_msgs::PointCloud2 &msg)
{
  

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr bag_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(msg, *bag_cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_msg(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(msg, *color_msg);
  for(int i = 0; i < color_msg->points.size(); i++){
    uint8_t r=255, g=0, b=0;
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    color_msg->points[i].rgb = *reinterpret_cast<float*>(&rgb);
  }

  cerr<<"original: "<<bag_cloud->points.size()<<endl;

  //=======passthrough filter=====================
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (bag_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.5, 1.5);
  pass.filter (*bag_cloud);
  cerr<<"filter1: "<<bag_cloud->points.size()<<endl;

  //=======voxel grid filter=====================
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  pcl::PCLPointCloud2::Ptr bag_cloud2 (new pcl::PCLPointCloud2 ());
  pcl::toPCLPointCloud2(*bag_cloud, *bag_cloud2);
  sor.setInputCloud (bag_cloud2);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*bag_cloud2);
  pcl::fromPCLPointCloud2(*bag_cloud2, *bag_cloud);
  //=======icp=====================
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  
  cerr<<"filter2: "<<bag_cloud->points.size()<<endl;
  icp.setInputSource(bag_cloud);
  icp.setInputTarget(pcl_cloud);
  pcl::PointCloud<pcl::PointXYZRGB> Final;
  std::cout << "icp start"<< std::endl;
  icp.align(Final, initial_guess);
  std::cout << "icp end"<< std::endl;

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  initial_guess = icp.getFinalTransformation();

  //
  tf::Matrix3x3 tf3d;
  tf3d.setValue((initial_guess(0,0)), (initial_guess(0,1)), (initial_guess(0,2)), 
        (initial_guess(1,0)), (initial_guess(1,1)), (initial_guess(1,2)), 
        (initial_guess(2,0)), (initial_guess(2,1)), (initial_guess(2,2)));
  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(initial_guess(0,3),initial_guess(1,3),initial_guess(2,3)));
  transform.setRotation(tfqt);
  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/world"));

  pcl::toROSMsg(Final, fin_cloud);
  
  fin_cloud.header.frame_id = "/world";
  fin_cloud.header.stamp = Time::now();
  pub_fin.publish(fin_cloud);
  //=======show map=====================
  map_cloud.header.frame_id = "/world";
  map_cloud.header.stamp = Time::now();
  pub_map.publish(map_cloud);

  //=======show bag=====================
  sensor_msgs::PointCloud2 m;
  pcl::toROSMsg(*color_msg, m);

  m.header.frame_id = "/world";
  m.header.stamp = Time::now();
  pub_bag.publish(m); 


}

int main (int argc, char** argv)
{ 
  ros::init(argc, argv, "hw5_node");
  ros::NodeHandle nh;
  Pointcloud pc(nh);
  ros::spin();
}