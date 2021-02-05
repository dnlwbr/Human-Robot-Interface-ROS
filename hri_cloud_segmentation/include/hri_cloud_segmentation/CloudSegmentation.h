//
// Created by weber on 15.06.20.
//

#ifndef HRI_CLOUD_SEGMENTATION_CLOUDSEGMENTATION_H
#define HRI_CLOUD_SEGMENTATION_CLOUDSEGMENTATION_H

#include "hri_cloud_segmentation/Segment.h"
#include <geometry_msgs/PointStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    void callback_gaze(geometry_msgs::PointStamped::ConstPtr const & msg);
    void initialize_transform(const std::string& source_frame);
    inline bool isInitialized() const { return isCloudInitialized && isGazeInitialized && isTransformInitialized;}

    void pass_through_filter();
    void voxel_filter();
    void planar_segmentation(double angle = 0);
    void min_cut_segmentation(double radius, bool show_background = false);
//    pcl::BoundingBoxXYZ bounding_box;

private:
    PointT gazeHitPoint;
    PointCloudT::ConstPtr cloud_incoming;

    bool isCloudInitialized = false;
    bool isGazeInitialized = false;
    bool isTransformInitialized = false;

    std::string target_frame;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    geometry_msgs::TransformStamped transformStamped;

    // Bounding box
//    void CalcBoundingBox();
};

#endif //HRI_CLOUD_SEGMENTATION_CLOUDSEGMENTATION_H
