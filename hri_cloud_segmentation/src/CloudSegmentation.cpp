//
// Created by weber on 15.06.20.
//

#include "../include/hri_cloud_segmentation/CloudSegmentation.h"


CloudSegmentation::CloudSegmentation()
    : cloud_incoming(new PointCloudT),
      cloud_segmented(new PointCloudT),
      tf_listener(tf_buffer) {
    //    bounding_box = pcl::BoundingBoxXYZ();
}


void CloudSegmentation::callback_cloud(PointCloudT::ConstPtr const & msg)
{
    cloud_incoming = msg;
    cloud_segmented = msg->makeShared();
    //cloud_segmented->header = msg->header;

    if (!isCloudInitialized)
    {
        isCloudInitialized = true;
        ROS_INFO("First cloud is initialized");
        target_frame = msg->header.frame_id;
    }
}


void CloudSegmentation::callback_gaze(geometry_msgs::PointStamped::ConstPtr const & msg)
{
    if (!isCloudInitialized)
        return;

    if (msg->header.frame_id != target_frame) {
        try{
            transformStamped = tf_buffer.lookupTransform(target_frame, msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("Failure %s\n", ex.what());
        }

        geometry_msgs::PointStamped msg_transformed;
        tf2::doTransform(*msg, msg_transformed, transformStamped);

        gazeHitPoint.x = msg_transformed.point.x;
        gazeHitPoint.y = msg_transformed.point.y;
        gazeHitPoint.z = msg_transformed.point.z;
    }
    else {
        gazeHitPoint.x = msg->point.x;
        gazeHitPoint.y = msg->point.y;
        gazeHitPoint.z = msg->point.z;
    }

    if (!isGazeInitialized)
    {
        isGazeInitialized = true;
        ROS_INFO("First gaze is initialized");
    }
}


void CloudSegmentation::pass_through_filter() {
    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_incoming);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 3.0);
    pass.filter(*cloud_segmented);
}


void CloudSegmentation::voxel_filter() {
    // Filtering input scan to increase speed.
    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setLeafSize(0.003f, 0.003f, 0.003f);
    voxel_grid.setInputCloud(cloud_segmented);
    voxel_grid.filter(*cloud_segmented);
}

/*void CloudSegmentation::CalcBoundingBox() {
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloud_segmented, minPoint, maxPoint);
//    bounding_box.x = (minPoint.x + maxPoint.x) / 2;
//    bounding_box.y = (minPoint.y + maxPoint.y) / 2;
//    bounding_box.z = (minPoint.z + maxPoint.z) / 2;
    bounding_box.width = maxPoint.x - minPoint.x;
    bounding_box.height = maxPoint.y - minPoint.y;
    bounding_box.depth = maxPoint.z - minPoint.z;
}*/


void CloudSegmentation::planar_segmentation() {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<PointT> extract;
    pcl::SACSegmentation<PointT> seg;

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    seg.setOptimizeCoefficients (true); // Optional

    int i = 0, nr_points = (int) cloud_segmented->size();
    // While 30% of the original cloud is still there
    while (cloud_segmented->size() > 0.5 * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_segmented);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.empty()) {
            ROS_DEBUG_STREAM("Could not estimate a planar model for the given dataset.");
            break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud_segmented);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_segmented);
        i++;
    }
}


void CloudSegmentation::min_cut_segmentation(double radius, bool show_background) {
    pcl::MinCutSegmentation<PointT> seg;
    seg.setInputCloud(cloud_segmented);

    PointCloudT::Ptr foreground_points(new PointCloudT);
    foreground_points->clear();
    foreground_points->points.push_back(gazeHitPoint);
    seg.setForegroundPoints(foreground_points);
    seg.setSigma(0.01); // Default: 0.25 (should be chosen depending on cloud resolution)
    seg.setRadius(radius);  // Default: 4
    seg.setNumberOfNeighbours(3); // Default: 14
    seg.setSourceWeight(1.25); // Default: 0.8

    std::vector<pcl::PointIndices> clusters;
    seg.extract (clusters);

    // Add points to a new cloud (clusters[0]: background, clusters[1]: object)
    foreground_points->clear();
    for (int indice : clusters[1].indices) {
        foreground_points->points.push_back(cloud_segmented->points[indice]);
    }
    cloud_segmented.swap(foreground_points);
    cloud_segmented->header = foreground_points->header;

    // Update cloud dimensions
    cloud_segmented->width = cloud_segmented->points.size();
    cloud_segmented->height = 1;
    cloud_segmented->is_dense = true;

    // Enable to show both, FG (white) and BG (red)
    if (show_background) {
        cloud_segmented = seg.getColoredCloud();
        cloud_segmented->header = cloud_incoming->header;
    }
}
