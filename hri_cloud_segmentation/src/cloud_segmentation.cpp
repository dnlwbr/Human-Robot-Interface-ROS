//
// Created by weber on 15.06.20.
//

#include "../include/hri_cloud_segmentation/CloudSegmentation.h"

#include <iostream>
#include "ros/ros.h"


int main (int argc, char** argv)
{
    ros::init(argc, argv, "hri_cloud_segmentation");
    ros::NodeHandle n;
    CloudSegmentation seg;

    std::string topic = "/points2";
    ros::Subscriber sub_cloud = n.subscribe<CloudSegmentation::PointCloudT>(topic, 1, &CloudSegmentation::callback_cloud, &seg);
    ROS_INFO("Subscribing to %s", topic.c_str());

    topic = "/hololens2/gaze/hitpoint";
    ros::Subscriber sub_gaze = n.subscribe<geometry_msgs::PointStamped>(topic, 1, &CloudSegmentation::callback_gaze, &seg);
    ROS_INFO("Subscribing to %s", topic.c_str());

    topic = "/points2/segmented";
    ros::Publisher pub_segmented_cloud = n.advertise<CloudSegmentation::PointCloudT>(topic, 1);
    ROS_INFO("Publisher for segmented cloud set to %s", topic.c_str());

    topic = "/points2/bounding_box_3d";
    ros::Publisher pub_box = n.advertise<vision_msgs::Detection3D>(topic, 1);
    ROS_INFO("Publisher for objects set to %s", topic.c_str());

    topic = "/hri_cloud_segmentation/visualization_marker";
    ros::Publisher pub_viz_marker = n.advertise<visualization_msgs::Marker>(topic, 1);
    ROS_INFO("Publisher for visualization marker set to %s", topic.c_str());

    //ros::ServiceServer service = n.advertiseService("/hri_cloud_segmentation/Segment", &MinCutSegmentation::callback_gaze, dynamic_cast<MinCutSegmentation*>(&seg));

    ROS_INFO("Waiting for initialization...");
    ros::Rate loop_rate(5);

    while (!seg.isInitialized()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    while (ros::ok())
    {
        seg.pass_through_filter();
        seg.voxel_filter();
        seg.planar_segmentation(30); // If epsilon angle equals 0 the axis is ignored.
        //seg.min_cut_segmentation(0.1, false);
        seg.clustering();
        seg.calc_bounding_box();
        pub_viz_marker.publish(seg.marker);
        pub_box.publish(seg.object);
        pub_segmented_cloud.publish(*seg.cloud_segmented);
        ros::spinOnce();    // process callbacks
        loop_rate.sleep();
    }

    return (0);
}