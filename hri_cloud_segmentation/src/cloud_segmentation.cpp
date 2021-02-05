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
    ROS_INFO("Subscribing to /points2");
    ros::Subscriber sub_cloud = n.subscribe<CloudSegmentation::PointCloudT>("/points2", 1, &CloudSegmentation::callback_cloud, &seg);
    ROS_INFO("Subscribing to /hololens2/gaze/hitpoint");
    ros::Subscriber sub_gaze = n.subscribe<geometry_msgs::PointStamped>("/hololens2/gaze/hitpoint", 1, &CloudSegmentation::callback_gaze, &seg);
    ROS_INFO("Publisher set to /points2/segmented");
    ros::Publisher pub = n.advertise<CloudSegmentation::PointCloudT>("/points2/segmented", 1);

    //ros::ServiceServer service = n.advertiseService("/hri_cloud_segmentation/Segment", &MinCutSegmentation::callback_gaze, dynamic_cast<MinCutSegmentation*>(&seg));

    ROS_INFO("Wait for initialization");
    ros::Rate loop_rate(5);

    while (!seg.isInitialized()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Publishing to /points2/segmented");

    while (ros::ok())
    {
        seg.pass_through_filter();
        seg.voxel_filter();
        seg.planar_segmentation(30); // If epsilon angle equals 0 the axis is ignored.
        //seg.min_cut_segmentation(0.1, false);
        pub.publish(*seg.cloud_segmented);
        ros::spinOnce();    // process callbacks
        loop_rate.sleep();
    }

    return (0);
}