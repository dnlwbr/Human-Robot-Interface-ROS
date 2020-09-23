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
    MinCutSegmentation seg;
    ROS_INFO("Subscribing to /HoloLens2/Gaze/HitPoint");
    ros::Subscriber sub_gaze = n.subscribe("/HoloLens2/Gaze/HitPoint", 1, &CloudSegmentation::callback_gaze, dynamic_cast<CloudSegmentation*>(&seg));
    ROS_INFO("Subscribing to /points2");
    ros::Subscriber sub_cloud = n.subscribe<CloudSegmentation::PointCloudT>("/points2", 1, &CloudSegmentation::callback_cloud, dynamic_cast<CloudSegmentation*>(&seg));
    ROS_INFO("Publishing to /points2/segmented");
    ros::Publisher pub = n.advertise<CloudSegmentation::PointCloudT>("/points2/segmented", 1);
    ros::Rate loop_rate(5);

    while (!seg.isInitialized()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    while (ros::ok())
    {
        seg.segment();
        pub.publish(*seg.cloud_segmented);
        ros::spinOnce();    // process callbacks
        loop_rate.sleep();
    }

    return (0);
}