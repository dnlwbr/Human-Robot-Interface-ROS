//
// Created by weber on 15.06.20.
//

#include "../include/hri_cloud_segmentation/CloudSegmentation.h"


CloudSegmentation::CloudSegmentation()
    : cloud_incoming(new PointCloudT),
      cloud_segmented(new PointCloudT),
      rgb_image_cropped_msg(new sensor_msgs::Image),
      rgb_camera_info(new sensor_msgs::CameraInfo),
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


void CloudSegmentation::callback_rgbImage(sensor_msgs::Image::ConstPtr const & img_msg, sensor_msgs::CameraInfo::ConstPtr const & info_msg)
{
    // Convert message to CvImage
    try
    {
        //rgb_image = cv_bridge::toCvCopy(img_msg, "bgr8");
        rgb_image = cv_bridge::toCvShare(img_msg);
        *rgb_camera_info = *info_msg;

        if (!isRGBImageInitialized) {
            isRGBImageInitialized = true;
            ROS_INFO("First RGB image is initialized");
            ROS_INFO("Camera Info received");
        }
    }
    catch (cv_bridge::Exception& e)
    {
        // ROS_ERROR("Could not convert from '%s' to 'bgr8'", img_msg->encoding.c_str());
        ROS_ERROR("cv_bridge exception: %s", e.what());
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


void CloudSegmentation::pass_through_filter(bool keepOrganized) {
    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_incoming);  // Start with original cloud
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 3.0);
    if (cloud_segmented->isOrganized() && keepOrganized) {
        pass.setKeepOrganized(true);
    }
    pass.filter(*cloud_segmented);
    UpdateProperties(*cloud_segmented);
}


void CloudSegmentation::voxel_filter(bool keepOrganized) {
    // Filtering input scan to increase speed.
    PointCloudT::Ptr cloud_voxel(new PointCloudT);
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setLeafSize(0.002f, 0.002f, 0.002f);
    voxel_grid.setInputCloud(cloud_segmented);

    if (cloud_segmented->isOrganized() && keepOrganized) {
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
        extract.setKeepOrganized(true);
        extract.filter(*cloud_segmented);
    }
    else
    {
        voxel_grid.filter(*cloud_segmented);
    }
    UpdateProperties(*cloud_segmented);
}


[[deprecated("Use voxel_grid instead, which provides better results.")]]
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


void CloudSegmentation::planar_segmentation(double angle, bool keepOrganized) {
    PointCloudT::Ptr  cloud_filtered(new PointCloudT);
    std::vector<int> indices_nonNan;
    // If cloud is not dense remove NaNs
    if (!cloud_segmented->is_dense) {
        pcl::removeNaNFromPointCloud(*cloud_segmented, *cloud_filtered, indices_nonNan);
    }
    else
    {
        cloud_filtered = cloud_segmented;
    }

    // Segment plane in filtered cloud
    pcl::SACSegmentation<PointT> seg;
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.003);
    seg.setInputCloud(cloud_filtered);
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

    // Map the inliers (plane in cloud_filtered) to the indices of cloud_segmented and extract corresponding points
    pcl::ExtractIndices<PointT> extract(false); // Initializing with true will allow getRemovedIndices()
    extract.setInputCloud(cloud_segmented);
    if (!cloud_segmented->is_dense) {
        pcl::PointIndices::Ptr inliers_mapped(new pcl::PointIndices());
        for (auto & idx : inliers->indices) {
            inliers_mapped->indices.push_back(indices_nonNan[idx]);
        }
        inliers = inliers_mapped;
    }
    extract.setIndices(inliers);

    extract.setNegative(true);
    if (cloud_segmented->isOrganized() && keepOrganized) {
        extract.setKeepOrganized(true);
    }
    extract.filter(*cloud_segmented);
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


void CloudSegmentation::clustering(bool keepOrganized) {
    // Cluster the point cloud
    pcl::search::KdTree<PointT>::Ptr kdtree_cluster(new pcl::search::KdTree<PointT>);
    kdtree_cluster->setInputCloud(cloud_segmented);

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.01);
    ec.setMinClusterSize(300);  // Caution: If the value is too small, small objects cannot be detected.
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(kdtree_cluster);
    ec.setInputCloud(cloud_segmented);
    // If cloud is not dense ignore NaNs
    if (!cloud_segmented->is_dense) {
        PointCloudT::Ptr  cloud_filtered(new PointCloudT);
        std::vector<int> indices_nonNan;
        pcl::removeNaNFromPointCloud(*cloud_segmented, *cloud_filtered, indices_nonNan);
        boost::shared_ptr<std::vector<int> > indicesPtr (new std::vector<int> (indices_nonNan));
        ec.setIndices(indicesPtr);
    }
    std::vector<pcl::PointIndices> clusters;
    ec.extract(clusters);

    // Vector[Vector[int]] -> Vector[int]
    pcl::IndicesPtr clusters_indices(new std::vector<int>);
    for (const auto & cluster : clusters)
    {
        clusters_indices->insert(std::end(*clusters_indices), std::begin(cluster.indices), std::end(cluster.indices));
    }

    // Search for the closest point among the cluster indices in the point cloud
    pcl::KdTreeFLANN<PointT> kdtree_closest_point;
    kdtree_closest_point.setInputCloud(cloud_segmented, clusters_indices);
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance;
    kdtree_closest_point.nearestKSearch(gazeHitPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

    // Find cluster with (approx.) gaze hit point
    pcl::PointIndices::Ptr clusterPtr(new pcl::PointIndices());  // Pointer to the cluster of the object
    // For each cluster found
    for (const auto & cluster : clusters)
    {
        // Check if point belongs to cluster (clusters[i] is i-th cluster)
        if (find(cluster.indices.begin(), cluster.indices.end(), pointIdxNKNSearch[0]) != cluster.indices.end()) {
            *clusterPtr = cluster;
            break;
        }
    }

    // Extract cluster
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_segmented);
    extract.setIndices(clusterPtr);
    extract.setNegative(false);
    if (cloud_segmented->isOrganized() && keepOrganized) {
        extract.setKeepOrganized(true);
    }
    extract.filter(*cloud_segmented);
    UpdateProperties(*cloud_segmented);
}


void CloudSegmentation::calc_bounding_box() {
    geometry_msgs::TransformStamped cloud_to_camera_base_leveled;
    geometry_msgs::TransformStamped camera_base_leveled_to_cloud;
    try{
        cloud_to_camera_base_leveled = tf_buffer.lookupTransform("azure_kinect_camera_base_leveled",
                                                                 cloud_segmented->header.frame_id,
                                                                 ros::Time(0),
                                                                 ros::Duration(1.0));
        camera_base_leveled_to_cloud = tf_buffer.lookupTransform(cloud_segmented->header.frame_id,
                                                                 "azure_kinect_camera_base_leveled",
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

    PointT minPoint3D, maxPoint3D;
    pcl::getMinMax3D(*cloud_segmented_leveled, minPoint3D, maxPoint3D);

    // Calculate 3D bounding box

    object.bbox.size.x = maxPoint3D.x - minPoint3D.x;
    object.bbox.size.y = maxPoint3D.y - minPoint3D.y;
    object.bbox.size.z = maxPoint3D.z - minPoint3D.z;

    geometry_msgs::Pose center;
    center.position.x = (minPoint3D.x + maxPoint3D.x) / 2;
    center.position.y = (minPoint3D.y + maxPoint3D.y) / 2;
    center.position.z = (minPoint3D.z + maxPoint3D.z) / 2;
    center.orientation.x = 0;
    center.orientation.y = 0;
    center.orientation.z = 0;
    center.orientation.w = 1;

    /*
    // For evaluation
    if (!isinf(object.bbox.size.x))
        object.bbox.center = center;
        ROS_INFO_STREAM(object.bbox);
    */

    tf2::doTransform(center, center, camera_base_leveled_to_cloud);
    object.bbox.center = center;
    pcl::toROSMsg(*cloud_segmented, object.source_cloud);

    // Fill header
    object.header.stamp = pcl_conversions::fromPCL(cloud_segmented->header.stamp);
    object.header.frame_id = cloud_segmented->header.frame_id;

    // Marker for visualization in RVIZ
    marker.header.frame_id = cloud_segmented->header.frame_id;
    marker.header.stamp = pcl_conversions::fromPCL(cloud_segmented->header.stamp);
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


    // Calculate 2D bounding box

    double inf = std::numeric_limits<double>::infinity();
    cv::Point2d minPoint2D(inf, inf);
    cv::Point2d maxPoint2D(-inf, -inf);

    if (cloud_segmented->empty()) {
        // If empty use full image
        minPoint2D.x = 0;
        minPoint2D.y = 0;
        maxPoint2D.x = rgb_image->image.cols;
        maxPoint2D.y = rgb_image->image.rows;
    }
    else
    {
        // Project all segmented point cloud points on 2D plane
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(rgb_camera_info);
        std::vector<cv::Point2d> points_projected;
        for (auto & point : cloud_segmented->points) {
            cv::Point3d point3D(point.x, point.y, point.z);
            cv::Point2d point2D = cam_model.project3dToPixel(point3D);
            points_projected.push_back(point2D);
        }

        // Calculate 2D corners
        for (auto & point : points_projected)
        {
            if (point.x < minPoint2D.x) { minPoint2D.x = point.x; }
            if (point.x > maxPoint2D.x) { maxPoint2D.x = point.x; }
            if (point.y < minPoint2D.y) { minPoint2D.y = point.y; }
            if (point.y > maxPoint2D.y) { maxPoint2D.y = point.y; }
        }
        if (minPoint2D.x < 0) { minPoint2D.x = 0; }
        if (maxPoint2D.x > rgb_image->image.cols) { maxPoint2D.x = rgb_image->image.cols; }
        if (minPoint2D.y < 0) { minPoint2D.y = 0; }
        if (maxPoint2D.y > rgb_image->image.rows) { maxPoint2D.y = rgb_image->image.rows; }
    }
    bbox_2d.clear(); bbox_2d.push_back(minPoint2D); bbox_2d.push_back(maxPoint2D);
//    ROS_INFO_STREAM(bbox_2d);
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


void CloudSegmentation::crop_image_to_bb() {
    int x = (int)bbox_2d[0].x;
    int y = (int)bbox_2d[0].y;
    int width = (int)(bbox_2d[1].x - bbox_2d[0].x);
    int height = (int)(bbox_2d[1].y - bbox_2d[0].y);

    // Crop image to 2D bounding box
    cv::Rect crop_region(x, y, width, height);
    cv_bridge::CvImage rgb_image_cropped = cv_bridge::CvImage(rgb_image->header, rgb_image->encoding);
    rgb_image->image(crop_region).copyTo(rgb_image_cropped.image);
    rgb_image_cropped_msg = rgb_image_cropped.toImageMsg();

    // Fill header
    // CvImage is constructed with header from image message
}


void CloudSegmentation::full_leveled_cloud_as_segmented() {
    geometry_msgs::TransformStamped cloud_to_camera_base_leveled;
    geometry_msgs::TransformStamped camera_base_leveled_to_cloud;
    try{
        cloud_to_camera_base_leveled = tf_buffer.lookupTransform("azure_kinect_camera_base_leveled",
                                                                 cloud_segmented->header.frame_id,
                                                                 ros::Time(0),
                                                                 ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Failure: %s", ex.what());
    }
    sensor_msgs::PointCloud2 cloud_incoming_msg;
    sensor_msgs::PointCloud2 cloud_incoming_leveled_msg;
    PointCloudT::Ptr cloud_segmented_leveled(new PointCloudT);
    pcl::toROSMsg(*cloud_incoming, cloud_incoming_msg);
    tf2::doTransform(cloud_incoming_msg, cloud_incoming_leveled_msg, cloud_to_camera_base_leveled);
    pcl::fromROSMsg(cloud_incoming_leveled_msg, *cloud_segmented);
}
