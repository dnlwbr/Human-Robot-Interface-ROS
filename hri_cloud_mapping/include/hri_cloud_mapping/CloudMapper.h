//
// Created by weber on 06.04.20.
//

#ifndef HRI_CLOUD_MAPPING_CLOUDMAPPER_H
#define HRI_CLOUD_MAPPING_CLOUDMAPPER_H

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/icp.h>


class CloudMapper
{
public:
    typedef pcl::PointXYZRGB PointT;
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
};

class ICPMapper: public CloudMapper
{
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    PointCloudT icp_cloud;

public:
    void setInput();
    inline void align() {icp.align(icp_cloud);}
    inline bool hasConverged() { return icp.hasConverged();}
    inline auto getFinalTransformation() { return icp.getFinalTransformation();}
};

//class NDTMapper:CloudMapper
//{
//    void setInput();
//    void align();
//};

#endif //HRI_CLOUD_MAPPING_CLOUDMAPPER_H
