//
// Created by weber on 06.04.20.
//

#ifndef HRI_CLOUD_MAPPING_CLOUDMAPPER_H
#define HRI_CLOUD_MAPPING_CLOUDMAPPER_H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

class CloudMapper
{
public:
//    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    void callback_human(PointCloudT::ConstPtr const & msg);
    void callback_robot(PointCloudT::ConstPtr const & msg);
    inline bool isInitialized() const { return isInitializedHC && isInitializedRC;}
    virtual void setInput() = 0;
    virtual void align() = 0;

protected:
    PointCloudT::ConstPtr human_cloud;
    PointCloudT::ConstPtr robot_cloud;
    bool isInitializedHC = false;
    bool isInitializedRC = false;
    PointCloudT tmp_cloud;
};


class ICPMapper: public CloudMapper
{
    pcl::IterativeClosestPoint<PointT, PointT> icp;

public:
    void setInput();
    inline void align() {icp.align(tmp_cloud);}
    inline bool hasConverged() { return icp.hasConverged();}
    inline auto getFitnessScore() { return icp.getFitnessScore();}
    inline auto getFinalTransformation() { return icp.getFinalTransformation();}
};


class NDTMapper: public CloudMapper
{
    PointCloudT::Ptr human_cloud_filtered {new PointCloudT };
    pcl::ApproximateVoxelGrid<PointT> approximate_voxel_filter;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

public:
    NDTMapper();
    void filter();
    void setInput();
    inline void align() {ndt.align(tmp_cloud);}
    inline bool hasConverged() { return ndt.hasConverged();}
    inline auto getFitnessScore() { return ndt.getFitnessScore();}
    inline auto getFinalTransformation() { return ndt.getFinalTransformation();}
};

#endif //HRI_CLOUD_MAPPING_CLOUDMAPPER_H
