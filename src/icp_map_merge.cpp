/**
1. Trigger when receive new map/tf (message filter new map and lidar scan)
2. use icp to match scan and merge into a new topic
**/
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <vector>
#include <string>
#include <math.h>
#include <signal.h>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
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
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

using namespace pcl;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::OccupancyGrid> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

class IcpMapping{
    public:
        IcpMapping();
        void pcCallback(const sensor_msgs::PointCloud2ConstPtr &pc, const nav_msgs::OccupancyGrid::ConstPtr &dummy);
        Eigen::Matrix4f get_transfrom(std::string link_name);

    private:
        ros::Publisher icp_pub;

        boost::shared_ptr<Sync> sync_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
        message_filters::Subscriber<nav_msgs::OccupancyGrid> grid_sub;

        ros::NodeHandle nh;
        ros::Subscriber sub;

        Eigen::Matrix4f map_trans;
        pcl::PointCloud<PointXYZ>::Ptr icp_map;
        pcl::PointCloud<PointXYZ>::Ptr pc_input;

        sensor_msgs::PointCloud2 ros_cloud_msg;

	    tf::TransformListener listener;
};

IcpMapping::IcpMapping(){
    icp_map.reset(new PointCloud<PointXYZ>());
    ros::Duration(1).sleep();
    icp_pub = nh.advertise<sensor_msgs::PointCloud2> ("/icp_mapping", 1);
    pc_input.reset(new PointCloud<PointXYZ>());

    map_trans = get_transfrom("velodyne1");

    pc_sub.subscribe(nh, "/husky1/velodyne1/velodyne_points", 1);
    grid_sub.subscribe(nh, "/map", 1);

    sync_.reset(new Sync(MySyncPolicy(10), pc_sub, grid_sub));
    sync_->registerCallback(boost::bind(&IcpMapping::pcCallback, this, _1, _2));
}

Eigen::Matrix4f IcpMapping::get_transfrom(std::string link_name){
	tf::StampedTransform transform;
	Eigen::Matrix4f trans;

	try{
		ros::Duration five_seconds(5.0);
		listener.waitForTransform("slam_map", link_name, ros::Time(0), five_seconds);
		listener.lookupTransform("slam_map", link_name, ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return trans;
	}
	Eigen::Quaternionf q(transform.getRotation().getW(), \
		transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ()); 
	Eigen::Matrix3f mat = q.toRotationMatrix();
	trans << mat(0,0), mat(0,1), mat(0,2), transform.getOrigin().getX(),
			mat(1,0), mat(1,1), mat(1,2), transform.getOrigin().getY(),
			mat(2,0), mat(2,1), mat(2,2), transform.getOrigin().getZ(),
			0, 0, 0, 1;
	return trans;
}

void IcpMapping::pcCallback(const sensor_msgs::PointCloud2ConstPtr &pc, const nav_msgs::OccupancyGrid::ConstPtr &dummy){
	ROS_INFO("mapping");
	fromROSMsg(*pc, *pc_input);
    map_trans = get_transfrom("velodyne1");
	pcl::transformPointCloud (*pc_input, *pc_input, map_trans);
    ROS_INFO("transformed");

    if (icp_map->empty()){
        ROS_INFO("Empty cloud");
        *icp_map += *pc_input;
    }
    else{
        ROS_INFO("Aligning");
        IterativeClosestPoint<PointXYZ, PointXYZ> icp;
        icp.setInputSource(pc_input);
        icp.setInputTarget(icp_map);

        icp.align(*pc_input);
        *icp_map += *pc_input;
        ROS_INFO("Score: %lf", icp.getFitnessScore());
    }

	toROSMsg(*icp_map, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "slam_map";
    ROS_INFO("Publishing");
    
	icp_pub.publish(ros_cloud_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_mapping");

    IcpMapping IM;

    ROS_INFO("\033[1;32m---->\033[0m Icp Mapping Started.");

    ros::spin();
    return 0;
}