//
// Created by weber on 15.06.20.
//

#ifndef HRI_CLOUD_SEGMENTATION_CLOUDSEGMENTATION_H
#define HRI_CLOUD_SEGMENTATION_CLOUDSEGMENTATION_H

#include "hri_cloud_segmentation/Segment.h"
#include <geometry_msgs/PointStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/min_cut_segmentation.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>


class CloudSegmentation
{
public:
    CloudSegmentation();
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    PointCloudT::Ptr cloud_segmented;
    void callback_cloud(PointCloudT::ConstPtr const & msg);
    virtual bool isInitialized() const = 0;
    virtual void segment() = 0;
//    pcl::BoundingBoxXYZ bounding_box;

protected:
    static bool isCloudInitialized;
    PointCloudT::ConstPtr cloud_incoming;
    PointCloudT::Ptr cloud_filtered;

    // Voxel filter
    pcl::VoxelGrid<PointT> voxel_filter;
    void filter();

    // Bounding box
//    void CalcBoundingBox();
};



class PlanarSegmentation: public CloudSegmentation
{
public:
    PlanarSegmentation();
    inline bool isInitialized() const { return isCloudInitialized;}
    void segment();

private:
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
    pcl::ExtractIndices<PointT> extract;
    PointCloudT::Ptr cloud_extracted;
};



class MinCutSegmentation: public CloudSegmentation
{
public:
    MinCutSegmentation();
    inline bool isInitialized() const { return isCloudInitialized && isGazeInitialized;}
    bool callback_gaze(hri_cloud_segmentation::Segment::Request &req,
                       hri_cloud_segmentation::Segment::Response &res);
    void segment();

private:
    PointT gazeHitPoint;
    bool isGazeInitialized = false;
    pcl::IndicesPtr indices;
    pcl::PassThrough<PointT> pass;
    pcl::MinCutSegmentation<PointT> seg;
    pcl::PointCloud<PointT>::Ptr foreground_points;
    float radius = 0.05f;
};

#endif //HRI_CLOUD_SEGMENTATION_CLOUDSEGMENTATION_H
