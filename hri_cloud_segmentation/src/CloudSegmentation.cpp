//
// Created by weber on 15.06.20.
//

#include "../include/hri_cloud_segmentation/CloudSegmentation.h"


bool CloudSegmentation::isCloudInitialized(false);


CloudSegmentation::CloudSegmentation()
    : cloud_incoming(new PointCloudT),
      cloud_filtered(new PointCloudT),
      cloud_segmented(new PointCloudT) {
    //    bounding_box = pcl::BoundingBoxXYZ();
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



MinCutSegmentation::MinCutSegmentation()
        : CloudSegmentation(),
          indices(new std::vector<int>),
          foreground_points(new PointCloudT)
{
}

bool MinCutSegmentation::callback_gaze(
        hri_cloud_segmentation::Segment::Request &req,
        hri_cloud_segmentation::Segment::Response &res)
{
    gazeHitPoint.x = req.gazeHitPoint.point.x;
    gazeHitPoint.y = req.gazeHitPoint.point.y;
    gazeHitPoint.z = req.gazeHitPoint.point.z;

    radius = req.radius;

    if (!isGazeInitialized)
    {
        isGazeInitialized = true;
        ROS_INFO("First gaze is initialized");
    }

    segment();
    ROS_INFO("New segmentation is published");

    if (!cloud_segmented->points.empty()) {
        res.success = true;
    } else {
        res.success = false;
    }
}

void MinCutSegmentation::segment() {
//    filter();

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*indices);

    seg.setInputCloud(cloud_filtered);
    seg.setIndices(indices);

    foreground_points->clear();
    cloud_segmented->clear();
    foreground_points->points.push_back(gazeHitPoint);
    seg.setForegroundPoints(foreground_points);
    seg.setSigma(0.01); // Default: 0.25 (should be chosen depending on cloud resolution)
    seg.setRadius(radius); // Default: 4
    seg.setNumberOfNeighbours(3); // Default: 14
    seg.setSourceWeight(1.25); // Default: 0.8

    std::vector<pcl::PointIndices> clusters;
    seg.extract (clusters);

//    std::cout << "Maximum flow is " << seg.getMaxFlow() << std::endl;

    // Add points to a new cloud (clusters[0]: background, clusters[1]: object)
    for (std::vector<int>::const_iterator point = clusters[1].indices.begin(); point != clusters[1].indices.end(); point++)
        cloud_segmented->points.push_back(cloud_filtered->points[*point]);

    // Update cloud dimensions
    cloud_segmented->width = cloud_segmented->points.size();
    cloud_segmented->height = 1;
    cloud_segmented->is_dense = true;

//    cloud_segmented = seg.getColoredCloud();
//    cloud_segmented->header = cloud_incoming->header; // Workaround...
}
