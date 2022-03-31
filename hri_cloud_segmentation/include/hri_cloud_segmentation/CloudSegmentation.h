//
// Created by weber on 15.06.20.
//

#ifndef HRI_CLOUD_SEGMENTATION_CLOUDSEGMENTATION_H
#define HRI_CLOUD_SEGMENTATION_CLOUDSEGMENTATION_H

#include "hri_cloud_segmentation/Segment.h"
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection3D.h>
#include <visualization_msgs/Marker.h>

#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include<opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


class CloudSegmentation
{
public:
    CloudSegmentation();
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    PointCloudT::Ptr cloud_segmented;
    sensor_msgs::Image::Ptr rgb_image_cropped_msg;


    void callback_cloud(PointCloudT::ConstPtr const & msg);
    void callback_gaze(geometry_msgs::PointStamped::ConstPtr const & msg);
    void callback_rgbImage(sensor_msgs::Image::ConstPtr const & img_msg, sensor_msgs::CameraInfo::ConstPtr const & info_msg);
    void initialize_transform(const std::string& source_frame);
    inline bool isInitialized() const { return isCloudInitialized && isGazeInitialized && isTransformInitialized
                                        && isRGBImageInitialized;}
    void reset_segmented_cloud();
    void UpdateProperties(PointCloudT &cloud);

    void pass_through_filter(float sx, float sy, float sz, bool keepOrganized = false);
    void voxel_filter(bool keepOrganized = false);
    void downsample();
    void planar_segmentation(double angle = 0, bool keepOrganized = false);
    void min_cut_segmentation(double radius, bool show_background = false);
    void clustering(bool keepOrganized = false);

    vision_msgs::Detection3D object;
    visualization_msgs::Marker marker;
    void calc_bounding_box();
    void crop_cloud_to_bb();
    void crop_image_to_bb();

    void full_leveled_cloud_as_segmented();

private:
    PointT gazeHitPoint;
    PointCloudT::ConstPtr cloud_incoming;
    boost::shared_ptr<const cv_bridge::CvImage> rgb_image;
    sensor_msgs::CameraInfo::Ptr rgb_camera_info;
    std::vector<cv::Point2d> bbox_2d;

    bool isCloudInitialized = false;
    bool isGazeInitialized = false;
    bool isRGBImageInitialized = false;
    bool isTransformInitialized = false;

    std::string target_frame;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    geometry_msgs::TransformStamped unity_origin_to_rgb_camera_link;
};

#endif //HRI_CLOUD_SEGMENTATION_CLOUDSEGMENTATION_H
