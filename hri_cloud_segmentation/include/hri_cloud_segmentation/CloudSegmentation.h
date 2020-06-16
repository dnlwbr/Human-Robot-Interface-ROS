//
// Created by weber on 15.06.20.
//

#ifndef HRI_CLOUD_SEGMENTATION_CLOUDSEGMENTATION_H
#define HRI_CLOUD_SEGMENTATION_CLOUDSEGMENTATION_H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>


class CloudSegmentation
{
public:
    //    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    PointCloudT::Ptr cloud_segmented;
    void callback(PointCloudT::ConstPtr const & msg);
    inline bool isInitialized() const { return isCloudInitialized;}
    CloudSegmentation();
    void segment();

private:
    bool isCloudInitialized = false;
    PointCloudT::ConstPtr cloud;
    PointCloudT::Ptr cloud_filtered;
    PointCloudT::Ptr cloud_extracted;

    // Segmentation
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
    pcl::SACSegmentation<PointT> seg;
    pcl::ExtractIndices<PointT> extract;

    // Voxel filter
    pcl::VoxelGrid<PointT> voxel_filter;
    void filter();
};

#endif //HRI_CLOUD_SEGMENTATION_CLOUDSEGMENTATION_H
