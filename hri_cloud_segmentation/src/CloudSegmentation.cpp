//
// Created by weber on 15.06.20.
//

#include "../include/hri_cloud_segmentation/CloudSegmentation.h"


void CloudSegmentation::callback(PointCloudT::ConstPtr const & msg) {
    cloud = msg;
    isCloudInitialized = true;
}

CloudSegmentation::CloudSegmentation()
    : coefficients(new pcl::ModelCoefficients()), inliers(new pcl::PointIndices()),
    cloud_filtered(new PointCloudT), cloud_extracted(new PointCloudT), cloud_segmented(new PointCloudT)
{
//    seg.setOptimizeCoefficients (true); // Optional
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
}

void CloudSegmentation::segment() {

    filter();

    cloud_segmented->header = cloud->header;

    int i = 0, nr_points = (int) cloud_filtered->points.size ();
    // While 30% of the original cloud is still there
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            ROS_DEBUG_STREAM("Could not estimate a planar model for the given dataset.");
            break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_extracted);
        std::cerr << "PointCloud representing the planar component: " << cloud_extracted->width * cloud_extracted->height << " data points." << std::endl;

        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_segmented);
        cloud_filtered.swap(cloud_segmented);
        i++;
    }
}

void  CloudSegmentation::filter() {
    // Filtering input scan to increase speed.
    voxel_filter.setLeafSize (0.01f, 0.01f, 0.01f);
    voxel_filter.setInputCloud(cloud);
    voxel_filter.filter(*cloud_filtered);
}