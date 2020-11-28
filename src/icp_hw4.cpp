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

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>

using namespace pcl;

//std::string PATH = "/home/argsubt/ros_docker_ws/hw4/map.pcd";
std::string PATH = "/home/argsubt/ros_docker_ws/localization_comp/SDC Localization Competition Data/map/itri_map.pcd";

class IcpMapping{
    public:
        IcpMapping();
        void pcCallback(const sensor_msgs::PointCloud2ConstPtr &pc);
        Eigen::Matrix4f get_transfrom(std::string link_name);
        Eigen::Matrix4f get_initial_guess();

    private:
        ros::NodeHandle nh;
        ros::Subscriber pc_sub;
        ros::Publisher pc_pub;
        ros::Publisher map_filtered_pub;
        ros::Publisher odom_pub;

        Eigen::Matrix4f trans;
        Eigen::Matrix4f init_trans;
        pcl::PointCloud<PointXYZI>::Ptr icp_map;
        pcl::PointCloud<PointXYZI>::Ptr icp_map_voxel;
        pcl::PointCloud<PointXYZI>::Ptr pc_input;
        pcl::PointCloud<PointXYZI>::Ptr pc_input_voxel;
        //pcl::PointCloud<PointXYZI>::Ptr pc_input_voxel;
        //pcl::PointCloud<PointXYZI>::Ptr pc_last_batch;

        sensor_msgs::PointCloud2 ros_cloud_msg;

	    tf::TransformListener listener;
        tf::TransformBroadcaster broadcaster;

        // PointCloud<PointXYZI>::Ptr downsample(PointCloud<PointXYZI>::Ptr &in);
        VoxelGrid<PointXYZI> sor;
};

IcpMapping::IcpMapping(){
    icp_map.reset(new PointCloud<PointXYZI>());
    pc_input.reset(new PointCloud<PointXYZI>());
    pc_input_voxel.reset(new PointCloud<PointXYZI>());
    icp_map_voxel.reset(new PointCloud<PointXYZI>());

    pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_trans", 1);
    map_filtered_pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_filtered", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    pcl::io::loadPCDFile<pcl::PointXYZI>(PATH, *icp_map);
    std::cout << "Loaded" << endl;

    std::cerr << "PointCloud before filtering: " << icp_map->width * icp_map->height 
        << " data points (" << pcl::getFieldsList (*icp_map) << ")." << std::endl;
    sor.setInputCloud(icp_map);
    sor.setLeafSize(0.5f, 0.5f, 0.5f);
    sor.filter(*icp_map_voxel);
    std::cerr << "PointCloud after filtering: " << icp_map_voxel->width * icp_map_voxel->height 
        << " data points (" << pcl::getFieldsList (*icp_map_voxel) << ")." << std::endl;

    toROSMsg(*icp_map_voxel, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "world";
    map_filtered_pub.publish(ros_cloud_msg);
    
    //ros::Duration(1).sleep();
    
    //pc_last_batch.reset(new PointCloud<PointXYZI>());

    //wait for gps
    std::cout << "waiting for gps" << std::endl;
    init_trans = get_initial_guess();
    std::cout << "initial guess get" << std::endl;
    std::cout << init_trans << std::endl;

    pc_sub = nh.subscribe("/lidar_points", 1, &IcpMapping::pcCallback, this);
}

// PointCloud<PointXYZI> IcpMapping::downsample(PointCloud<PointXYZI>::Ptr &in){
//     PointCloud<PointXYZI>::Ptr pc_filtered;
    
//     sor.setInputCloud(in);
//     sor.setLeafSize(0.01f, 0.01f, 0.01f);
//     sor.filter(*pc_filtered);
    
//     return *pc_filtered;
// }

Eigen::Matrix4f IcpMapping::get_initial_guess(){
    Eigen::Matrix4f trans;
    tf::Transform transform;
    
    geometry_msgs::PointStampedConstPtr gps_point;
    gps_point = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/gps", nh);

    tf::Quaternion q;
    q.setRPY(0, 0, -3.8078303612);
    Eigen::Quaternionf eq(q.w(), q.x(), q.y(), q.z());
	Eigen::Matrix3f mat = eq.toRotationMatrix();

    trans << mat(0,0), mat(0,1), mat(0,2), (*gps_point).point.x,
			mat(1,0), mat(1,1), mat(1,2), (*gps_point).point.y,
			mat(2,0), mat(2,1), mat(2,2), (*gps_point).point.z,
			0, 0, 0, 1;
	return trans;
}

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
	ROS_INFO("localizing");
	fromROSMsg(*pc, *pc_input);

    //=======cropbox filter=========================
    CropBox<PointXYZI> crop;

    crop.setMin(Eigen::Vector4f(-30,-15,-10,1.0)); //给定立体空间
    crop.setMax(Eigen::Vector4f(30,15,10,1.0));  //数据随意给的，具体情况分析
    crop.setInputCloud(pc_input);
    crop.setKeepOrganized(true);
    crop.setUserFilterValue(0.1f);
    crop.filter(*pc_input);

    trans = get_transfrom("velodyne");
    std::cout << init_trans << std::endl;
	transformPointCloud (*pc_input, *pc_input, trans);
    transformPointCloud (*pc_input, *pc_input, init_trans);
    
    ROS_INFO("transformed to world");

    //=======cropbox filter=========================
    // CropBox

    //=======passthrough filter=====================
    // PassThrough<PointXYZI> pass;
    // pass.setInputCloud (pc_input);
    // pass.setFilterFieldName ("z");
    // pass.setFilterLimits (-0.5, 1.5);
    // pass.filter (*pc_input);

    sor.setInputCloud(pc_input);
    sor.setLeafSize(0.7f, 0.7f, 0.7f);
    sor.filter(*pc_input_voxel);

    // toROSMsg(*pc_input_voxel, ros_cloud_msg);
	// ros_cloud_msg.header.frame_id = "world";
    // pc_pub.publish(ros_cloud_msg);

    if (icp_map_voxel->empty()){
        ROS_INFO("Empty cloud");
    }
    else{
        ROS_INFO("Aligning");
        IterativeClosestPoint<PointXYZI, PointXYZI> icp;
        icp.setInputSource(pc_input_voxel);
        icp.setInputTarget(icp_map_voxel);
        icp.setMaximumIterations (800);
        icp.setTransformationEpsilon (1e-9);
        icp.setRANSACOutlierRejectionThreshold (0.05);
        icp.setMaxCorrespondenceDistance (5);
        //icp.setEuclideanFitnessEpsilon (0.01);
        icp.align(*pc_input_voxel);
        //transformPointCloud (*pc_input, *pc_input, (icp.getFinalTransformation()));
        // *icp_map += *pc_input;
        // sor.setInputCloud(icp_map_voxel);
        // sor.setLeafSize(0.2f, 0.2f, 0.2f);
        // sor.filter(*icp_map_voxel);

        //copyPointCloud(*pc_input, *pc_last);
        ROS_INFO("Score: %lf", icp.getFitnessScore());
        init_trans = (icp.getFinalTransformation()) * init_trans ;
        std::cout << init_trans << std::endl;
    }

    ROS_INFO("Publishing");

    toROSMsg(*pc_input_voxel, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "world";
    pc_pub.publish(ros_cloud_msg);

    tf::Transform world_trans;
    Eigen::Affine3f affine_trans;
    affine_trans.matrix() = init_trans;
    tf::transformEigenToTF(affine_trans.cast<double>(), world_trans);
    broadcaster.sendTransform(tf::StampedTransform(world_trans, ros::Time::now(), "world", "base_link"));

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world";
 
    //set the position
    odom.pose.pose.position.x = world_trans.getOrigin().getX();
    odom.pose.pose.position.y = world_trans.getOrigin().getY();
    odom.pose.pose.position.z = world_trans.getOrigin().getZ();

    geometry_msgs::Quaternion quat_msg;
    quaternionTFToMsg(world_trans.getRotation() , quat_msg);
    odom.pose.pose.orientation = quat_msg;

    //publish the message
    odom_pub.publish(odom);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_mapping");

    IcpMapping IM;

    ROS_INFO("\033[1;32m---->\033[0m Icp Mapping Started.");

    ros::spin();
    return 0;
}