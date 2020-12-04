/**
1. Trigger when receive new map/tf (message filter new map and lidar scan)
2. use icp to match scan and merge into a new topic
**/
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

using namespace pcl;

double last_timestamp = 1556114916.473415;
int frame_num = 201;
double initial_x = -285.456721951;
double initial_y = 225.77162962;
double initial_z = -12.4146628257;
double initial_yaw = -3.8078303612;

std::string baseFrame = "base_link";
std::string lidarFrame = "velodyne";

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
        std::ofstream outputFile;
        int id = 1;

        Eigen::Matrix4f trans_lidar2base;
        Eigen::Matrix4f init_trans;
        Eigen::Matrix4f inverse_trans;
        pcl::PointCloud<PointXYZI>::Ptr icp_map;
        pcl::PointCloud<PointXYZI>::Ptr icp_map_voxel;
        pcl::PointCloud<PointXYZI>::Ptr cropped_map;
        pcl::PointCloud<PointXYZI>::Ptr pc_input;
        pcl::PointCloud<PointXYZI>::Ptr pc_input_voxel;
        //pcl::PointCloud<PointXYZI>::Ptr pc_input_voxel;
        pcl::PointCloud<PointXYZI>::Ptr pc_last;

        sensor_msgs::PointCloud2 ros_cloud_msg;

	    tf::TransformListener listener;
        tf::TransformBroadcaster broadcaster;

        cbshot cb;

        // PointCloud<PointXYZI>::Ptr downsample(PointCloud<PointXYZI>::Ptr &in);
        VoxelGrid<PointXYZI> sor;
};

IcpMapping::IcpMapping(){
    icp_map.reset(new PointCloud<PointXYZI>());
    pc_input.reset(new PointCloud<PointXYZI>());
    pc_input_voxel.reset(new PointCloud<PointXYZI>());
    icp_map_voxel.reset(new PointCloud<PointXYZI>());
    cropped_map.reset(new PointCloud<PointXYZI>());
    pc_last.reset(new PointCloud<PointXYZI>());

    pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_trans", 1);
    map_filtered_pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_cropped", 1, true);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    // pcl::io::loadPCDFile<pcl::PointXYZI>(PATH, *icp_map);
    std::cerr << "Waiting for map" << endl;
    ros_cloud_msg = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/map_filtered", nh));
    fromROSMsg(ros_cloud_msg, *icp_map_voxel);


    // std::cerr << "PointCloud before filtering: " << icp_map->width * icp_map->height 
    //     << " data points (" << pcl::getFieldsList (*icp_map) << ")." << std::endl;
    // sor.setInputCloud(icp_map);
    // sor.setLeafSize(0.35f, 0.35f, 0.35f);
    // sor.filter(*icp_map_voxel);

    //=======passthrough filter=====================
    PassThrough<PointXYZI> pass;
    pass.setInputCloud (icp_map_voxel);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-20, 20);
    pass.filter (*icp_map_voxel);

    // CropBox<PointXYZI> crop;

    // crop.setMin(Eigen::Vector4f(-2000,-2000,1.5,1.0)); //给定立体空间
    // crop.setMax(Eigen::Vector4f(2000,2000,20,1.0));  //数据随意给的，具体情况分析
    // crop.setInputCloud(icp_map_voxel);
    // crop.setKeepOrganized(true);
    // crop.setUserFilterValue(0.1f);
    // crop.setNegative(false);
    // crop.filter(*icp_map_voxel);
    // std::cerr << "PointCloud after filtering: " << icp_map_voxel->width * icp_map_voxel->height 
    //     << " data points (" << pcl::getFieldsList (*icp_map_voxel) << ")." << std::endl;

    toROSMsg(*icp_map_voxel, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "world";
    map_filtered_pub.publish(ros_cloud_msg);

    // cb.cloud2 = *icp_map;
    // cb.calculate_normals_map(0.5);
    // std::vector<int> indices;
    // pcl::removeNaNNormalsFromPointCloud(cb.cloud2_normals, cb.cloud2_normals, indices);

    // boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (indices));
    // pcl::ExtractIndices<PointXYZI> eifilter (true); // Initializing with true will allow us to extract the removed indices
    // eifilter.setInputCloud (cb.cloud2.makeShared());
    // eifilter.setIndices (indicesptr);
    // eifilter.filter (cb.cloud2);

    // cb.calculate_voxel_grid_map(0.8);
    // cb.calculate_SHOT_map(4);
    
    //ros::Duration(1).sleep();
    outputFile.open("result2.csv");
    outputFile << "id,x,y,z,yaw,pitch,roll\n";

    //pc_last_batch.reset(new PointCloud<PointXYZI>());
    //wait for gps
    std::cout << "waiting for gps" << std::endl;
    init_trans = get_initial_guess();
    std::cerr << "initial guess get" << std::endl;
    std::cerr << init_trans << std::endl;

    pc_sub = nh.subscribe("/lidar_points", 400, &IcpMapping::pcCallback, this);
    trans_lidar2base = get_transfrom(lidarFrame);
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
    //gps_point = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/gps", nh);

    tf::Quaternion q;
    q.setRPY(0, 0, initial_yaw);
    Eigen::Quaternionf eq(q.w(), q.x(), q.y(), q.z());
	Eigen::Matrix3f mat = eq.toRotationMatrix();

    // trans << mat(0,0), mat(0,1), mat(0,2), (*gps_point).point.x,
	// 		mat(1,0), mat(1,1), mat(1,2), (*gps_point).point.y,
	// 		mat(2,0), mat(2,1), mat(2,2), (*gps_point).point.z,
	// 		0, 0, 0, 1;

    trans << mat(0,0), mat(0,1), mat(0,2),  initial_x,
			mat(1,0), mat(1,1), mat(1,2), initial_y,
			mat(2,0), mat(2,1), mat(2,2), initial_z,
			0, 0, 0, 1;
	return trans;
}

Eigen::Matrix4f IcpMapping::get_transfrom(std::string link_name){
	tf::StampedTransform transform;
	Eigen::Matrix4f trans;

	try{
		ros::Duration five_seconds(5.0);
		listener.waitForTransform(baseFrame, link_name, ros::Time(0), five_seconds);
		listener.lookupTransform(baseFrame, link_name, ros::Time(0), transform);
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
	// ROS_INFO("Map to BL");
    // transformPointCloud (*icp_map_voxel, *cropped_map, init_trans.inverse());
    
    CropBox<PointXYZI> crop;
    // crop.setMin(Eigen::Vector4f(-30,-15,1.5,1.0)); //给定立体空间
    // crop.setMax(Eigen::Vector4f(30,15,20,1.0));  //数据随意给的，具体情况分析
    // crop.setInputCloud(cropped_map);
    // crop.setKeepOrganized(true);
    // crop.setUserFilterValue(0.1f);
    // crop.setNegative(false);
    // crop.filter(*cropped_map);

    toROSMsg(*pc_last, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "world";
    //pc_pub.publish(ros_cloud_msg);
    
    // transformPointCloud (*cropped_map, *cropped_map, init_trans);

    ROS_INFO("localizing");
	ros::Time timestamp = pc->header.stamp;
    fromROSMsg(*pc, *pc_input);

    transformPointCloud (*pc_input, *pc_input, trans_lidar2base);
    // if (id != 1){
    //     Eigen::Matrix4f tmp;
    //     tmp << 1, 0, 0, 10,
    //             0, 1, 0, 0,
    //             0, 0, 1, 0,
    //             0, 0, 0, 1;
    //     transformPointCloud (*pc_input, *pc_input, tmp);
    // }
    //=======cropbox filter=========================
    
    //crop outside
    crop.setMin(Eigen::Vector4f(-100,-100,0.1,1.0)); //给定立体空间
    crop.setMax(Eigen::Vector4f(100,100,20,1.0));  //数据随意给的，具体情况分析
    crop.setInputCloud(pc_input);
    crop.setKeepOrganized(true);
    crop.setUserFilterValue(0.1f);
    crop.setNegative(false);
    crop.filter(*pc_input);

    // //crop inside
    // crop.setMin(Eigen::Vector4f(-30,-3,-10,1.0)); //给定立体空间
    // crop.setMax(Eigen::Vector4f(30,12,30,1.0));  //数据随意给的，具体情况分析
    // crop.setInputCloud(pc_input);
    // crop.setKeepOrganized(true);
    // crop.setUserFilterValue(0.1f);
    // crop.setNegative(true);
    // crop.filter(*pc_input);

    std::cout << init_trans << std::endl;
    transformPointCloud (*pc_input, *pc_input, init_trans);
    
    ROS_INFO("transformed to world");
    
    //=======cropbox filter=========================
    // SHOT

    sor.setInputCloud(pc_input);
    sor.setLeafSize(0.3f, 0.3f, 0.3f);
    sor.filter(*pc_input_voxel);

    toROSMsg(*pc_input_voxel, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "world";
    pc_pub.publish(ros_cloud_msg);
    // ros::Duration(5).sleep();
    // cb.cloud1 = *pc_input;
    // cb.calculate_normals_cloud(0.5);
    // std::vector<int> indices;
    // pcl::removeNaNNormalsFromPointCloud(cb.cloud1_normals, cb.cloud1_normals, indices);
    
    // boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (indices));
    // pcl::ExtractIndices<PointXYZI> eifilter (true); // Initializing with true will allow us to extract the removed indices
    // eifilter.setInputCloud (cb.cloud1.makeShared());
    // eifilter.setIndices (indicesptr);
    // eifilter.filter (cb.cloud1);
    
    // cb.calculate_voxel_grid_cloud(0.8);
    // cb.calculate_SHOT_cloud(4);

    // printf("inf1");
    // typename pcl::PointCloud<SHOT352>::iterator itr;
    // for (itr = cb.cloud1_shot.begin(); itr != cb.cloud1_shot.end(); itr++)
    // {
    //     std::cout << *itr << std::endl;
    // }
    // printf("inf2");

    // pcl::Correspondences corresp;
    // pcl::registration::CorrespondenceEstimation<pcl::SHOT352, pcl::SHOT352> shot_corr;
    // shot_corr.setInputSource(cb.cloud1_shot.makeShared());
    // shot_corr.setInputTarget(cb.cloud2_shot.makeShared());
    // std::cout << "Status : Calculated SHOT descriptors and finding the correspondences" << endl;
    // cout << "May take a while...depends on the feature descriptor and its support size" << endl;
    // shot_corr.determineReciprocalCorrespondences(corresp);


    // pcl::CorrespondencesConstPtr correspond = boost::make_shared< pcl::Correspondences >(corresp);

    // std::cout << "RANSAC rejections" << endl;
    // pcl::Correspondences corr;
    // pcl::registration::CorrespondenceRejectorSampleConsensus< pcl::PointXYZI > Ransac_based_Rejection;
    // Ransac_based_Rejection.setInputSource(cb.cloud1_keypoints.makeShared());
    // Ransac_based_Rejection.setInputTarget(cb.cloud2_keypoints.makeShared());
    // Ransac_based_Rejection.setInputCorrespondences(correspond);
    // Ransac_based_Rejection.getCorrespondences(corr);

    // Eigen::Matrix4f ransac_mat = Ransac_based_Rejection.getBestTransformation();
    // cout << "RANSAC based Transformation Matrix : \n" << ransac_mat << endl;

    // pcl::transformPointCloud(*pc_input, *pc_input, ransac_mat);

    // toROSMsg(*pc_input, ros_cloud_msg);
	// ros_cloud_msg.header.frame_id = "world";
    // pc_pub.publish(ros_cloud_msg);

    //=======passthrough filter=====================
    // PassThrough<PointXYZI> pass;
    // pass.setInputCloud (pc_input);
    // pass.setFilterFieldName ("z");
    // pass.setFilterLimits (-0.5, 1.5);
    // pass.filter (*pc_input);


    // toROSMsg(*pc_input_voxel, ros_cloud_msg);
	// ros_cloud_msg.header.frame_id = "world";
    // pc_pub.publish(ros_cloud_msg);
    Eigen::MatrixXd ICP_COV(6,6);
    if (icp_map_voxel->empty()){
        ROS_INFO("Empty cloud");
    }
    else{
        ROS_INFO("Aligning");
        IterativeClosestPoint<PointXYZI, PointXYZI> icp;
        if (id != 1){
            icp.setInputSource(pc_input_voxel);
            icp.setInputTarget(pc_last);
            icp.setMaximumIterations (100);
            icp.setTransformationEpsilon (1e-09);
            icp.setRANSACOutlierRejectionThreshold (0.08);
            icp.setMaxCorrespondenceDistance (8);
            //icp.setEuclideanFitnessEpsilon (0.01);
            // Eigen::Matrix4f guess;
            // guess << 1, 0, 0, 0.05,
            //          0, 1, 0, 0.0,
            //          0, 0, 1, 0.0,
            //          0, 0, 0, 1.0;
            icp.align(*pc_input_voxel);
            ROS_INFO("Score: %lf", icp.getFitnessScore());
            init_trans = (icp.getFinalTransformation()) * init_trans ;
        }
        //transformPointCloud (*pc_input, *pc_input, (icp.getFinalTransformation()));
        // *icp_map += *pc_input;
        // sor.setInputCloud(icp_map_voxel);
        // sor.setLeafSize(0.2f, 0.2f, 0.2f);
        // sor.filter(*icp_map_voxel);

        //copyPointCloud(*pc_input, *pc_last);
        icp.setInputSource(pc_input_voxel);
        icp.setInputTarget(icp_map_voxel);
        icp.setMaximumIterations (200);
        icp.setTransformationEpsilon (1e-09);
        icp.setRANSACOutlierRejectionThreshold (0.08);
        icp.setMaxCorrespondenceDistance (8);
        icp.align(*pc_input_voxel);
        ROS_INFO("Score: %lf", icp.getFitnessScore());
        init_trans = (icp.getFinalTransformation()) * init_trans ;
        std::cout << init_trans << std::endl;
        copyPointCloud(*pc_input_voxel, *pc_last);
        Eigen::Matrix4f final_trans = icp.getFinalTransformation();
        ICP_COV = Eigen::MatrixXd::Zero(6,6);
        calculate_ICP_COV(*pc_input_voxel, *icp_map_voxel, final_trans, ICP_COV);
    }
    Eigen::Matrix3f rot;
	rot << init_trans(0,0), init_trans(0,1), init_trans(0,2),
			init_trans(1,0), init_trans(1,1), init_trans(1,2),
			init_trans(2,0), init_trans(2,1), init_trans(2,2);
    Eigen::Quaternionf quat(rot);
    quat.normalize();
    Eigen::Matrix3f mats = quat.toRotationMatrix();
	init_trans << mats(0,0), mats(0,1), mats(0,2), init_trans(0,3),
			mats(1,0), mats(1,1), mats(1,2), init_trans(1,3),
			mats(2,0), mats(2,1), mats(2,2), init_trans(2,3),
			0, 0, 0, 1;


    ROS_INFO("Publishing");

    toROSMsg(*pc_input_voxel, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "world";
    pc_pub.publish(ros_cloud_msg);

    tf::Transform world_trans;
    Eigen::Affine3f affine_trans;
    affine_trans.matrix() = init_trans;
    tf::transformEigenToTF(affine_trans.cast<double>(), world_trans);
    broadcaster.sendTransform(tf::StampedTransform(world_trans, ros::Time::now(), "world", baseFrame));

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

    boost::array<double, 36> covariance;
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            covariance[i*6+j] = ICP_COV(i,j);
    odom.pose.covariance = covariance;

    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(world_trans.getRotation());
    mat.getEulerYPR(yaw, pitch, roll);

    ROS_INFO("%d, %lf", id, timestamp.toSec());
    //write to csv
    outputFile << id++ << "," << odom.pose.pose.position.x << ","<< odom.pose.pose.position.y << ","<< odom.pose.pose.position.z << ","<< yaw << ","<< pitch << ","<< roll << "\n";
    std::cout << timestamp.toSec() << "," << odom.pose.pose.position.x << ","<< odom.pose.pose.position.y << ","<< odom.pose.pose.position.z << ","<< yaw << ","<< pitch << ","<< roll << std::endl;

    // if (std::abs(timestamp.toSec() - last_timestamp) <= 0.001){
    //     ROS_INFO("close: %lf, %lf",timestamp.toSec(), last_timestamp);
    //     outputFile.close();
    // }

    if (id > frame_num){
        ROS_INFO("close: %lf, %lf",timestamp.toSec(), last_timestamp);
        outputFile.close();
    }


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