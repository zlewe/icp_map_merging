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
#include <pcl/registration/gicp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

using namespace pcl;

auto BATCH_SIZE = 16;

class IcpMapping{
    public:
        IcpMapping();
        void pcCallback(const sensor_msgs::PointCloud2ConstPtr &pc);
        Eigen::Matrix4f get_transfrom(std::string link_name);

    private:
        int count;
        ros::Publisher icp_pub;
        ros::Publisher pc_pub;

        ros::NodeHandle nh;
        ros::Subscriber pc_sub;

        Eigen::Matrix4f trans;
        pcl::PointCloud<PointXYZI>::Ptr icp_map;
        pcl::PointCloud<PointXYZI>::Ptr icp_map_voxel;
        pcl::PointCloud<PointXYZI>::Ptr pc_input;
        pcl::PointCloud<PointXYZI>::Ptr pc_input_voxel;
        //pcl::PointCloud<PointXYZI>::Ptr pc_input_voxel;
        //pcl::PointCloud<PointXYZI>::Ptr pc_last_batch;

        sensor_msgs::PointCloud2 ros_cloud_msg;

	    tf::TransformListener listener;

        // PointCloud<PointXYZI>::Ptr downsample(PointCloud<PointXYZI>::Ptr &in);
        VoxelGrid<PointXYZI> sor;
};

IcpMapping::IcpMapping(){
    icp_map.reset(new PointCloud<PointXYZI>());
    ros::Duration(1).sleep();
    icp_pub = nh.advertise<sensor_msgs::PointCloud2> ("/icp_mapping", 1);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_trans", 1);
    pc_input.reset(new PointCloud<PointXYZI>());
    pc_input_voxel.reset(new PointCloud<PointXYZI>());
    icp_map_voxel.reset(new PointCloud<PointXYZI>());
    //pc_last_batch.reset(new PointCloud<PointXYZI>());

    trans = get_transfrom("velodyne1");

    pc_sub = nh.subscribe("/husky1/velodyne1/velodyne_points", 1, &IcpMapping::pcCallback, this);
}

// PointCloud<PointXYZI> IcpMapping::downsample(PointCloud<PointXYZI>::Ptr &in){
//     PointCloud<PointXYZI>::Ptr pc_filtered;
    
//     sor.setInputCloud(in);
//     sor.setLeafSize(0.01f, 0.01f, 0.01f);
//     sor.filter(*pc_filtered);
    
//     return *pc_filtered;
// }

Eigen::Matrix4f IcpMapping::get_transfrom(std::string link_name){
	tf::StampedTransform transform;
	Eigen::Matrix4f trans;

	try{
		ros::Duration five_seconds(5.0);
		listener.waitForTransform("base_link", link_name, ros::Time(0), five_seconds);
		listener.lookupTransform("base_link", link_name, ros::Time(0), transform);
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

void IcpMapping::pcCallback(const sensor_msgs::PointCloud2ConstPtr &pc){
	ROS_INFO("mapping");
	fromROSMsg(*pc, *pc_input);
	transformPointCloud (*pc_input, *pc_input, trans);
    ROS_INFO("transformed");

    toROSMsg(*pc_input, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "icp_map";
    pc_pub.publish(ros_cloud_msg);
    
    sor.setInputCloud(pc_input);
    sor.setLeafSize(0.2f, 0.2f, 0.2f);
    sor.filter(*pc_input_voxel);

    if (icp_map->empty()){
        ROS_INFO("Empty cloud");
        *icp_map += *pc_input;
        sor.setInputCloud(icp_map);
        sor.setLeafSize(0.2f, 0.2f, 0.2f);
        sor.filter(*icp_map_voxel);
        //copyPointCloud(*pc_input, *pc_last);
    }
    else{
        ROS_INFO("Aligning");
        GeneralizedIterativeClosestPoint<PointXYZI, PointXYZI> icp;
        icp.setInputSource(pc_input_voxel);
        icp.setInputTarget(icp_map_voxel);
        icp.setMaximumIterations (100);
        icp.setTransformationEpsilon (1e-10);
        icp.setRANSACOutlierRejectionThreshold (1);
        icp.setMaxCorrespondenceDistance (100);
        icp.setEuclideanFitnessEpsilon (0.001);

        icp.align(*pc_input_voxel);
        *icp_map_voxel += *pc_input_voxel;
        //transformPointCloud (*pc_input, *pc_input, (icp.getFinalTransformation()));
        //*icp_map += *pc_input;
        sor.setInputCloud(icp_map_voxel);
        sor.setLeafSize(0.2f, 0.2f, 0.2f);
        sor.filter(*icp_map_voxel);

        //copyPointCloud(*pc_input, *pc_last);
        ROS_INFO("Score: %lf", icp.getFitnessScore());
        trans = (icp.getFinalTransformation()) * trans;
    }

    

	toROSMsg(*icp_map_voxel, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "icp_map";
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