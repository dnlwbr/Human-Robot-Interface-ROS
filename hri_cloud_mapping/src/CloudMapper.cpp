//
// Created by weber on 06.04.20.
//

#include "../include/hri_cloud_mapping/CloudMapper.h"

//#include <iostream>


void CloudMapper::callback_human(PointCloudT::ConstPtr const & msg) {
    human_cloud = msg;
    isInitializedHC = true;
//    ROS_INFO("cb_h");
// //    pcl::PointCloud<pcl::PointNormal>::iterator itr;
//    pcl::PointCloud<pcl::PointXYZ>::iterator itr;
//    for (itr = human_cloud->begin(); itr != human_cloud->end(); itr++)
//    {
//        if (!pcl::isFinite(*itr))
//        {
//            printf("Found a point with inf!");
//            std::cout << itr->x << "," << itr->y << "," << itr->z << std::endl;
//        }
//        if ((!pcl_isfinite(itr->normal_x)) ||
//            (!pcl_isfinite(itr->normal_y)) ||
//            (!pcl_isfinite(itr->normal_z)))
//        {
//            printf("Found a point normal with inf!");
//            std::cout << itr->normal_x << "," << itr->normal_y << ","
//                 << itr->normal_z << std::endl;
//        }
//    }
}

void CloudMapper::callback_robot(PointCloudT::ConstPtr const & msg) {
    robot_cloud = msg;
    isInitializedRC = true;
//    ROS_INFO("cb_r");
}

void ICPMapper::setInput() {
    // Iterative Closest Point algorithm
    icp.setInputSource(human_cloud);
    icp.setInputTarget(robot_cloud);
}

NDTMapper::NDTMapper() {
    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon (0.01);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize (0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution (1.0);
    // Setting max number of registration iterations.
    ndt.setMaximumIterations (35);
}

void  NDTMapper::filter() {
    // Filtering input scan to roughly 10% of original size to increase speed of registration.
    approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud(human_cloud);
    approximate_voxel_filter.filter(*human_cloud_filtered);
}

void NDTMapper::setInput() {
    // Apply filter
    filter();
    // Setting point cloud to be aligned.
    ndt.setInputSource(human_cloud_filtered);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget(robot_cloud);
}
