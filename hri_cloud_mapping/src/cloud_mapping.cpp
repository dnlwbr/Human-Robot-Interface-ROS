//
// Created by weber on 02.04.20.
//

#include "../include/hri_cloud_mapping/CloudMapper.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

//#include <pcl/io/pcd_io.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "hri_cloud_mapping");
    ros::NodeHandle n;
    ICPMapper mapper;
    ros::Subscriber human_points_sub = n.subscribe<ICPMapper::PointCloudT>("/kinect2/hd/points2", 1, &ICPMapper::callback_human,
                                                                           dynamic_cast<CloudMapper*>(&mapper));
    ros::Subscriber robot_points_sub = n.subscribe<ICPMapper::PointCloudT>("/kinect2/hd/points", 1, &ICPMapper::callback_robot, dynamic_cast<CloudMapper*>(&mapper));
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/hri_cloud_mapping/robot_gaze", 1);
    ROS_INFO("Publishing robot gaze to /hri_cloud_mapping/robot_gaze");
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (mapper.isInitialized()) {
            mapper.setInput();
            mapper.align();

            if (mapper.hasConverged ()) {
                std::cout << mapper.getFinalTransformation() << std::endl;
    //        chatter_pub.publish(msg);
            }
        }
        ros::spinOnce();    // process callbacks
        loop_rate.sleep();
    }


    return 0;
}