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

    if (!isTransformInitialized) {
        initialize_transform(msg->header.frame_id);
    }
    // Check again
    if (isTransformInitialized) {
        geometry_msgs::PointStamped msg_transformed = *msg;
        tf2::doTransform(*msg, msg_transformed, transformStamped);
        gazeHitPoint.x = msg_transformed.point.x;
        gazeHitPoint.y = msg_transformed.point.y;
        gazeHitPoint.z = msg_transformed.point.z;

        if (!isGazeInitialized) {
            isGazeInitialized = true;
            ROS_INFO("First gaze is initialized");
        }
    }
}


void CloudSegmentation::initialize_transform(const std::string& source_frame) {
    try{
        transformStamped = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
        isTransformInitialized = true;
        ROS_INFO("Transformation is initialized");
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Failure: %s", ex.what());
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


void CloudSegmentation::planar_segmentation(double angle) {
    pcl::SACSegmentation<PointT> seg;
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(500);
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(cloud_segmented);
    seg.setOptimizeCoefficients (true); // Optional

    Eigen::Vector3f axis = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    Eigen::Quaternionf rotation = Eigen::Quaternionf(transformStamped.transform.rotation.w,
                                                     transformStamped.transform.rotation.x,
                                                     transformStamped.transform.rotation.y,
                                                     transformStamped.transform.rotation.z);
    axis = rotation.inverse() * axis;
    seg.setAxis(axis.normalized());
    seg.setEpsAngle(angle * (M_PI/180.0f) );

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    seg.segment(*inliers, *coefficients);

    // Extract the inliers
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_segmented);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_segmented);
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


void CloudSegmentation::clustering() {
    // Search for the closest point in point cloud
    pcl::KdTreeFLANN<PointT> kdtree_closest_point;
    kdtree_closest_point.setInputCloud(cloud_segmented);
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance;
    kdtree_closest_point.nearestKSearch(gazeHitPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

    // Cluster the point cloud
    pcl::search::KdTree<PointT>::Ptr kdtree_cluster(new pcl::search::KdTree<PointT>);
    kdtree_cluster->setInputCloud(cloud_segmented);

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.01);
    ec.setMinClusterSize(1000);
    ec.setMaxClusterSize(99999000);
    ec.setSearchMethod(kdtree_cluster);
    ec.setInputCloud(cloud_segmented);

    std::vector<pcl::PointIndices> clusters;
    ec.extract(clusters);

    // Find closes cluster and add points to a new cloud (clusters[i] is i-th cluster)
    PointCloudT::Ptr foreground_points(new PointCloudT);
    foreground_points->clear();
    for (const auto & cluster : clusters)
    {
        if (find(cluster.indices.begin(), cluster.indices.end(), pointIdxNKNSearch[0]) != cluster.indices.end())
        {
            for (int indice : cluster.indices)
            {
                foreground_points->points.push_back(cloud_segmented->points[indice]);
            }
            break;
        }
    }
    cloud_segmented.swap(foreground_points);
    cloud_segmented->header = foreground_points->header;

    // Update cloud dimensions
    cloud_segmented->width = cloud_segmented->points.size();
    cloud_segmented->height = 1;
    cloud_segmented->is_dense = true;
}

/*void CloudSegmentation::calc_bounding_box() {
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloud_segmented, minPoint, maxPoint);
//    bounding_box.x = (minPoint.x + maxPoint.x) / 2;
//    bounding_box.y = (minPoint.y + maxPoint.y) / 2;
//    bounding_box.z = (minPoint.z + maxPoint.z) / 2;
    bounding_box.width = maxPoint.x - minPoint.x;
    bounding_box.height = maxPoint.y - minPoint.y;
    bounding_box.depth = maxPoint.z - minPoint.z;
}*/
