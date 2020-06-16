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
    CloudSegmentation cs;
    ros::Subscriber sub = n.subscribe<CloudSegmentation::PointCloudT>("/points2", 1, &CloudSegmentation::callback, &cs);
    ROS_INFO("Subscribing to /points2");
    ros::Publisher pub = n.advertise<CloudSegmentation::PointCloudT>("/points2/segmented", 1);
    ROS_INFO("Publishing to /points2/segmented");
    ros::Rate loop_rate(5);

    while (!cs.isInitialized()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    while (ros::ok())
    {
        cs.segment();
        pub.publish(*cs.cloud_segmented);
        ros::spinOnce();    // process callbacks
        loop_rate.sleep();
    }

    return (0);
}