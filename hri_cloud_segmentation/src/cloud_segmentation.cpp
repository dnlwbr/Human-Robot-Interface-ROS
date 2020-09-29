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
    ROS_INFO("Subscribing to /points2");
    ros::Subscriber sub_cloud = n.subscribe<CloudSegmentation::PointCloudT>("/points2", 1, &CloudSegmentation::callback_cloud, dynamic_cast<CloudSegmentation*>(&seg));
    ROS_INFO("Start segmentation server");
    ros::ServiceServer service = n.advertiseService("/hri_cloud_segmentation/Segment", &MinCutSegmentation::callback_gaze, dynamic_cast<MinCutSegmentation*>(&seg));
    ros::Publisher pub = n.advertise<CloudSegmentation::PointCloudT>("/points2/segmented", 1);
    ROS_INFO("Wait for initialization");
    ros::Rate loop_rate(5);

    while (!seg.isInitialized()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Publishing to /points2/segmented");

    while (ros::ok())
    {
        pub.publish(*seg.cloud_segmented);
        ros::spinOnce();    // process callbacks
        loop_rate.sleep();
    }

    return (0);
}