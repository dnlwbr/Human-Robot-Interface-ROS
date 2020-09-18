//
// Created by weber on 15.06.20.
//

#ifndef HRI_CLOUD_SEGMENTATION_CLOUDSEGMENTATION_H
#define HRI_CLOUD_SEGMENTATION_CLOUDSEGMENTATION_H

#include <geometry_msgs/PointStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/lccp_segmentation.h>
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
    void callback_gaze(geometry_msgs::PointStamped::ConstPtr const & msg);
    void callback_cloud(PointCloudT::ConstPtr const & msg);
    inline bool isInitialized() const { return isGazeInitialized && isCloudInitialized;}
    virtual void segment() = 0;

protected:
    bool isGazeInitialized = false;
    bool isCloudInitialized = false;
    PointT GazeHitPoint;
    PointCloudT::ConstPtr cloud_incoming;
    PointCloudT::Ptr cloud_filtered;

    // Voxel filter
    pcl::VoxelGrid<PointT> voxel_filter;
    void filter();
};



class PlanarSegmentation: public CloudSegmentation
{
public:
    PlanarSegmentation();
    void segment();

private:
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
    pcl::ExtractIndices<PointT> extract;
    PointCloudT::Ptr cloud_extracted;
};



class EuclideanSegmentation: public CloudSegmentation
{
public:
    EuclideanSegmentation();
    void segment();
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented;

private:
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
    pcl::ExtractIndices<PointT> extract;
    PointCloudT::Ptr cloud_extracted;
    std::vector<pcl::PointIndices> cluster_indices;
//    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    pcl::EuclideanClusterExtraction<PointT> ec;
};



class RGSegmentation: public CloudSegmentation
{
public:
    RGSegmentation();
    void segment();

private:
    pcl::NormalEstimation<PointT , pcl::Normal> normal_estimator;
    pcl::search::Search<PointT>::Ptr tree;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    pcl::IndicesPtr indices;
    pcl::PassThrough<PointT> pass;
    pcl::RegionGrowing<PointT, pcl::Normal> reg;
    std::vector <pcl::PointIndices> clusters;
    PointCloudT::Ptr colored_cloud;
};



class CRGSegmentation: public CloudSegmentation
{
public:
    CRGSegmentation();
    void segment();

private:
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree;
    pcl::IndicesPtr indices;
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    std::vector<pcl::PointIndices> clusters;
};



class LCCPSegmentation: public CloudSegmentation
{
public:
    typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;
    LCCPSegmentation();
    void segment();
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_segmented;
    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud;

private:
    pcl::NormalEstimation<PointT , pcl::Normal> normal_estimator;
    pcl::search::Search<PointT>::Ptr tree;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;

    // Supervoxel Stuff
    std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
    std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
    pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud;
    LCCPSegmentation::SuperVoxelAdjacencyList sv_adjacency_list;
    float voxel_resolution = 0.005f;
    float seed_resolution = 0.02f;
    float color_importance = 0.0f;
    float spatial_importance = 1.0f;
    float normal_importance = 4.0f;
    bool use_single_cam_transform = false;
    bool use_supervoxel_refinement = false;

    // LCCPSegmentation Stuff
    float concavity_tolerance_threshold = 10;
    float smoothness_threshold = 0.1;
    std::uint32_t min_segment_size = 0;
    bool use_extended_convexity = false;
    bool use_sanity_criterion = false;

    float normals_scale = seed_resolution / 2.0;
    unsigned int k_factor = 0;
};



class MinCutSegmentation: public CloudSegmentation
{
public:
    MinCutSegmentation();
    void segment();

private:
    pcl::IndicesPtr indices;
    pcl::PassThrough<PointT> pass;
    pcl::MinCutSegmentation<PointT> seg;
    pcl::PointCloud<PointT>::Ptr foreground_points;
};

#endif //HRI_CLOUD_SEGMENTATION_CLOUDSEGMENTATION_H
