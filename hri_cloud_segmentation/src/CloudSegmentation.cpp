//
// Created by weber on 15.06.20.
//

#include "../include/hri_cloud_segmentation/CloudSegmentation.h"


CloudSegmentation::CloudSegmentation()
    : cloud_incoming(new PointCloudT),
      cloud_segmented(new PointCloudT),
      tf_listener(tf_buffer) {
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
        tf2::doTransform(*msg, msg_transformed, unity_origin_to_rgb_camera_link);
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
        unity_origin_to_rgb_camera_link = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
        isTransformInitialized = true;
        ROS_INFO("Transformation is initialized");
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Failure: %s", ex.what());
    }
}


void CloudSegmentation::UpdateProperties(PointCloudT &cloud) {
    if (cloud.isOrganized()) {
        cloud.is_dense = false; // Cloud contains NaN (or Inf) values
    }
    else
    {
        cloud.width = cloud_segmented->points.size();
        cloud.height = 1;
        cloud.is_dense = true;
    }
}


void CloudSegmentation::pass_through_filter() {
    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_incoming);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 3.0);
    pass.setKeepOrganized(true);
    pass.filter(*cloud_segmented);

    UpdateProperties(*cloud_segmented);
}


void CloudSegmentation::voxel_filter(bool keepOrganized) {
    // Filtering input scan to increase speed.
    PointCloudT::Ptr cloud_voxel(new PointCloudT);
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setLeafSize(0.003f, 0.003f, 0.003f);
    voxel_grid.setInputCloud(cloud_segmented);

    if (keepOrganized) {
        voxel_grid.filter(*cloud_voxel);

        // Search for the respective closest point in original cloud
        pcl::KdTreeFLANN<PointT> kdtree_closest_point;
        kdtree_closest_point.setInputCloud(cloud_segmented);
        int K = 1;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance;
        pcl::PointIndices::Ptr indicesPtr(new pcl::PointIndices());
        for (auto & it : *cloud_voxel) {
            kdtree_closest_point.nearestKSearch(it, K, pointIdxNKNSearch, pointNKNSquaredDistance);
            indicesPtr->indices.push_back(pointIdxNKNSearch[0]);
        }

        // Extract closest point from cloud
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_segmented);
        extract.setIndices(indicesPtr);
        extract.setNegative(false);
        if (cloud_segmented->isOrganized()) {
            extract.setKeepOrganized(true);
        }
        extract.filter(*cloud_segmented);
    }
    else
    {
        voxel_grid.filter(*cloud_segmented);
    }
    UpdateProperties(*cloud_segmented);
}


void CloudSegmentation::downsample() {
    int scale = 20;
    pcl::PointIndices::Ptr indicesPtr(new pcl::PointIndices());
    for (int i = 0; i < cloud_segmented->size()/scale; i++){
        indicesPtr->indices.push_back(i * scale);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_segmented);
    extract.setIndices(indicesPtr);
    extract.setNegative(false);
    if (cloud_segmented->isOrganized()) {
        extract.setKeepOrganized(true);
    }
    extract.filter(*cloud_segmented);
    UpdateProperties(*cloud_segmented);
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
    Eigen::Quaternionf rotation = Eigen::Quaternionf(unity_origin_to_rgb_camera_link.transform.rotation.w,
                                                     unity_origin_to_rgb_camera_link.transform.rotation.x,
                                                     unity_origin_to_rgb_camera_link.transform.rotation.y,
                                                     unity_origin_to_rgb_camera_link.transform.rotation.z);
    axis = rotation.inverse() * axis;
    seg.setAxis(axis.normalized());
    seg.setEpsAngle(angle * (M_PI/180.0f) );

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    seg.segment(*inliers, *coefficients);

    // Extract the inliers
    pcl::ExtractIndices<PointT> extract(true); // Initializing with true will allow getRemovedIndices()
    extract.setInputCloud(cloud_segmented);
    extract.setIndices(inliers);

    extract.setNegative(true);
    if (cloud_segmented->isOrganized()) {
        extract.setKeepOrganized(true);
    }
    extract.filter(*cloud_segmented);
    background_indices = extract.getRemovedIndices();
    UpdateProperties(*cloud_segmented);
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

    UpdateProperties(*cloud_segmented);

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

    std::vector<pcl::PointIndices> clusters;
    pcl::PointIndices::Ptr clusterPtr(new pcl::PointIndices());  // Pointer to the cluster of the object
    // Prefer a faster method if the cloud is organized, over EuclidanClusterExtraction
    if (cloud_segmented->isOrganized()) {
        pcl::EuclideanClusterComparator<PointT , pcl::Normal, pcl::Label>::Ptr comparator(new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>());
        comparator->setDistanceThreshold(0.005f,false);
        comparator->setInputCloud(cloud_segmented);
        // Set the entire scene to false, and the inliers of the objects located on top of the plane to true
        pcl::Label l{}; l.label = 1;
        pcl::PointCloud<pcl::Label>::Ptr scene(new pcl::PointCloud<pcl::Label>(cloud_segmented->width, cloud_segmented->height, l));
        // Mask the objects that we want to split into clusters
        for (const int &index : *background_indices)
            scene->points[index].label = 0;
        comparator->setLabels(scene);

        std::vector<bool> exclude_labels {true, false};
        comparator->setExcludeLabels (exclude_labels);

        pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> segmenter(comparator);
        segmenter.setInputCloud(cloud_segmented);
        pcl::PointCloud<pcl::Label> euclidean_labels;
        segmenter.segment(euclidean_labels, clusters);
    }
    else
    {
        // Cluster the point cloud
        pcl::search::KdTree<PointT>::Ptr kdtree_cluster(new pcl::search::KdTree<PointT>);
        kdtree_cluster->setInputCloud(cloud_segmented);

        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.005);
        ec.setMinClusterSize(1000);
        ec.setMaxClusterSize(99999000);
        ec.setSearchMethod(kdtree_cluster);
        ec.setInputCloud(cloud_segmented);
        ec.extract(clusters);
    }

    // For each cluster found
    for (const auto & cluster : clusters)
    {
        // Check if point belongs to cluster (clusters[i] is i-th cluster)
        if (find(cluster.indices.begin(), cluster.indices.end(), pointIdxNKNSearch[0]) != cluster.indices.end()) {
            *clusterPtr = cluster;
            break;
        }
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_segmented);
    extract.setIndices(clusterPtr);
    extract.setNegative(false);
    if (cloud_segmented->isOrganized()) {
        extract.setKeepOrganized(true);
    }
    extract.filter(*cloud_segmented);
    UpdateProperties(*cloud_segmented);
}


void CloudSegmentation::calc_bounding_box() {
    geometry_msgs::TransformStamped cloud_to_camera_base_leveled;
    geometry_msgs::TransformStamped camera_base_leveled_to_cloud;
    try{
        cloud_to_camera_base_leveled = tf_buffer.lookupTransform("camera_base_leveled",
                                                                 cloud_segmented->header.frame_id,
                                                                 ros::Time(0),
                                                                 ros::Duration(1.0));
        camera_base_leveled_to_cloud = tf_buffer.lookupTransform(cloud_segmented->header.frame_id,
                                                                 "camera_base_leveled",
                                                                 ros::Time(0),
                                                                 ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Failure: %s", ex.what());
    }
    sensor_msgs::PointCloud2 cloud_segmented_msg;
    sensor_msgs::PointCloud2 cloud_segmented_leveled_msg;
    PointCloudT::Ptr cloud_segmented_leveled(new PointCloudT);
    pcl::toROSMsg(*cloud_segmented, cloud_segmented_msg);
    tf2::doTransform(cloud_segmented_msg, cloud_segmented_leveled_msg, cloud_to_camera_base_leveled);
    pcl::fromROSMsg(cloud_segmented_leveled_msg, *cloud_segmented_leveled);

    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloud_segmented_leveled, minPoint, maxPoint);

    object.bbox.size.x = maxPoint.x - minPoint.x;
    object.bbox.size.y = maxPoint.y - minPoint.y;
    object.bbox.size.z = maxPoint.z - minPoint.z;

    geometry_msgs::Pose center;
    center.position.x = (minPoint.x + maxPoint.x) / 2;
    center.position.y = (minPoint.y + maxPoint.y) / 2;
    center.position.z = (minPoint.z + maxPoint.z) / 2;
    center.orientation.x = 0;
    center.orientation.y = 0;
    center.orientation.z = 0;
    center.orientation.w = 1;

    tf2::doTransform(center, center, camera_base_leveled_to_cloud);
    object.bbox.center = center;
    pcl::toROSMsg(*cloud_segmented, object.source_cloud);

    // Fill header
    object.header.stamp = ros::Time::now();
    object.header.frame_id = cloud_segmented->header.frame_id;

    // Marker for visualization in RVIZ
    marker.header.frame_id = cloud_segmented->header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "hri_cloud_segmentation";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = center.position;
    marker.pose.orientation = center.orientation;
    marker.scale = object.bbox.size;
    marker.color.a = 0.7; // alpha value
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
}

void CloudSegmentation::crop_cloud_to_bb() {
    pcl::CropBox<PointT> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(object.bbox.center.position.x - object.bbox.size.x/2,
                                     object.bbox.center.position.y - object.bbox.size.y/2,
                                     object.bbox.center.position.z - object.bbox.size.z/2,
                                     1.0));
    boxFilter.setMax(Eigen::Vector4f(object.bbox.center.position.x + object.bbox.size.x/2,
                                     object.bbox.center.position.y + object.bbox.size.y/2,
                                     object.bbox.center.position.z + object.bbox.size.z/2,
                                     1.0));

    Eigen::Quaternionf orientation = Eigen::Quaternionf(object.bbox.center.orientation.w,
                                                     object.bbox.center.orientation.x,
                                                     object.bbox.center.orientation.y,
                                                     object.bbox.center.orientation.z);

    auto euler = orientation.toRotationMatrix().eulerAngles(0,1,2);
    //boxFilter.setRotation(Eigen::Vector3f(euler[0], euler[1], euler[2]));
    boxFilter.setRotation(Eigen::Vector3f(0,0,0));

    boxFilter.setInputCloud(cloud_incoming);
    boxFilter.filter(*cloud_segmented);
    //?UpdateProperties(*cloud_segmented);
    //pcl::toROSMsg(*cloud_segmented, object.source_cloud);
}
