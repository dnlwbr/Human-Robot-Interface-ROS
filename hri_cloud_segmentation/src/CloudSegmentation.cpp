//
// Created by weber on 15.06.20.
//

#include "../include/hri_cloud_segmentation/CloudSegmentation.h"


CloudSegmentation::CloudSegmentation()
    : cloud_filtered(new PointCloudT),
      cloud_segmented(new PointCloudT) {
}

void CloudSegmentation::callback_gaze(geometry_msgs::PointStamped::ConstPtr const & msg)
{
    GazeHitPoint.x = msg->point.x;
    GazeHitPoint.y = msg->point.y;
    GazeHitPoint.z = msg->point.z;

    if (!isGazeInitialized)
    {
        isGazeInitialized = true;
        ROS_INFO("First gaze is initialized");
    }
}

void CloudSegmentation::callback_cloud(PointCloudT::ConstPtr const & msg)
{
    cloud_incoming = msg;
    *cloud_filtered = *cloud_incoming;
    cloud_segmented->header = cloud_incoming->header;

    if (!isCloudInitialized)
    {
        isCloudInitialized = true;
        ROS_INFO("First cloud is initialized");
    }
}

void  CloudSegmentation::filter() {
    // Filtering input scan to increase speed.
    voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel_filter.setInputCloud(cloud_incoming);
    voxel_filter.filter(*cloud_filtered);
}



PlanarSegmentation::PlanarSegmentation()
    : CloudSegmentation(),
      cloud_extracted(new PointCloudT),
      coefficients(new pcl::ModelCoefficients()),
      inliers(new pcl::PointIndices())
{
    seg.setOptimizeCoefficients (true); // Optional
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
}

void PlanarSegmentation::segment() {
    filter();

    int i = 0, nr_points = (int) cloud_filtered->size();
    // While 30% of the original cloud is still there
    while (cloud_filtered->size() > 0.3 * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            ROS_DEBUG_STREAM("Could not estimate a planar model for the given dataset.");
            break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_extracted);

        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_segmented);
        cloud_filtered.swap(cloud_segmented);
        i++;
    }
}



EuclideanSegmentation::EuclideanSegmentation()
        : CloudSegmentation(),
          cloud_extracted(new PointCloudT),
          coefficients(new pcl::ModelCoefficients()),
          inliers(new pcl::PointIndices())
{
    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true); // Optional
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
}

void EuclideanSegmentation::segment() {
    filter();

    int i=0, nr_points = (int) cloud_filtered->size();
    while (cloud_filtered->size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            ROS_DEBUG_STREAM("Could not estimate a planar model for the given dataset.");
            break;
        }

        // Extract the planar inliers from the input cloud
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        // Get the points associated with the planar surface
        extract.filter(*cloud_extracted);

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_segmented);
        *cloud_filtered = *cloud_segmented;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_filtered);

    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        PointCloudT ::Ptr cloud_cluster (new PointCloudT);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

//        TODO: IF GAZE POINT...
        *cloud_segmented = *cloud_cluster;
        cloud_segmented->header = cloud_incoming->header; // Workaround...
        j++;
    }
}



RGSegmentation::RGSegmentation()
        : CloudSegmentation(),
          tree(new pcl::search::KdTree<PointT>),
          cloud_normals(new pcl::PointCloud<pcl::Normal>),
          indices(new std::vector <int>)
{
    reg.setMinClusterSize(300);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);
}

void RGSegmentation::segment() {
//    filter();

    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud_incoming);
    normal_estimator.setKSearch(50);
//    ne.setRadiusSearch (0.03)
    normal_estimator.compute(*cloud_normals);

    pass.setInputCloud(cloud_incoming);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*indices);

    reg.setInputCloud(cloud_incoming);
    //reg.setIndices (indices);
    reg.setInputNormals(cloud_normals);

    reg.extract(clusters);

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;

    colored_cloud = reg.getColoredCloud();
    *cloud_segmented = *colored_cloud;
    cloud_segmented->header = cloud_incoming->header; // Workaround...
}



CRGSegmentation::CRGSegmentation()
        : CloudSegmentation(),
          tree (new pcl::search::KdTree<pcl::PointXYZRGB>),
          indices(new std::vector <int>)
{
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);

    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(5);
    reg.setPointColorThreshold(3);
    reg.setRegionColorThreshold(2);
    reg.setMinClusterSize(100);
}

void CRGSegmentation::segment() {
    filter();

    pass.setInputCloud(cloud_filtered);
    pass.filter(*indices);

    reg.setInputCloud(cloud_filtered);
    reg.setIndices(indices);

    reg.extract(clusters);

    cloud_segmented = reg.getColoredCloud();
    cloud_segmented->header = cloud_incoming->header; // Workaround...

}



LCCPSegmentation::LCCPSegmentation()
        : CloudSegmentation(),
          cloud_segmented(new pcl::PointCloud<pcl::PointXYZL>),
          cloud_normals(new pcl::PointCloud<pcl::Normal>),
          tree(new pcl::search::KdTree<PointT>)
{
    if (use_extended_convexity)
        k_factor = 1;
}

void LCCPSegmentation::segment() {

    /// Normal estimation
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud_incoming);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*cloud_normals);

    /// Preparation of Input: Supervoxel Oversegmentation
    pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
//    super.setUseSingleCameraTransform(use_single_cam_transform);
    super.setInputCloud(cloud_incoming);
//    super.makeSupervoxelNormalCloud();
    super.setNormalCloud(cloud_normals);
    super.setColorImportance(color_importance);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);

    /// Extracting supervoxels
    super.extract(supervoxel_clusters);

    if (use_supervoxel_refinement) {
        PCL_INFO ("Refining supervoxels\n");
        super.refineSupervoxels(2, supervoxel_clusters);
    }

    /// Getting supervoxel adjacency
    super.getSupervoxelAdjacency(supervoxel_adjacency);

    /// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
    sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud(
            supervoxel_clusters);

    /// The Main Step: Perform LCCPSegmentation

    /// Segmentation
    pcl::LCCPSegmentation<PointT> lccp;
    lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
    lccp.setSanityCheck(use_sanity_criterion);
    lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
    lccp.setKFactor(k_factor);
    lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
    lccp.setMinSegmentSize(min_segment_size);
    lccp.segment();

    /// Interpolation voxel cloud -> input cloud and relabeling
    sv_labeled_cloud = super.getLabeledCloud();
    cloud_segmented = sv_labeled_cloud->makeShared();
    lccp.relabelCloud(*cloud_segmented);
    lccp.getSVAdjacencyList(sv_adjacency_list);  // Needed for visualization
}



MinCutSegmentation::MinCutSegmentation()
        : CloudSegmentation(),
          indices(new std::vector<int>),
          foreground_points(new PointCloudT)
{
}

void MinCutSegmentation::segment() {
    filter();

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*indices);

    seg.setInputCloud(cloud_filtered);
    seg.setIndices(indices);

    foreground_points->clear();
    foreground_points->points.push_back(GazeHitPoint);
    seg.setForegroundPoints(foreground_points);

    seg.setSigma(0.01); // Default: 0.25 (should be chosen depending on cloud resolution)
    seg.setRadius(0.05); // Default: 4
    seg.setNumberOfNeighbours(3); // Default: 14
    seg.setSourceWeight(1.5); // Default: 0.8

    std::vector<pcl::PointIndices> clusters;
    seg.extract (clusters);

    std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

    cloud_segmented = seg.getColoredCloud();
    cloud_segmented->header = cloud_incoming->header; // Workaround...
}
