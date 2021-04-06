//
// Created by weber on 15.06.20.
//

#include "../include/hri_cloud_segmentation/CloudSegmentation.h"

#include <iostream>
#include "ros/ros.h"
#include <image_transport/image_transport.h>


int main (int argc, char** argv)
{
    ros::init(argc, argv, "hri_cloud_segmentation");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    CloudSegmentation seg;

    ROS_INFO("----- Subscriber -----");

    std::string topic = "/points2";
    ros::Subscriber sub_cloud = nh.subscribe<CloudSegmentation::PointCloudT>(topic, 1, &CloudSegmentation::callback_cloud, &seg);
    ROS_INFO("Subscribing to %s", topic.c_str());

    topic = "/hololens2/gaze/hitpoint";
    ros::Subscriber sub_gaze = nh.subscribe<geometry_msgs::PointStamped>(topic, 1, &CloudSegmentation::callback_gaze, &seg);
    ROS_INFO("Subscribing to %s", topic.c_str());

    //topic = "/rgb/image_raw";
    topic = "/rgb/image_rect_color";  // PinholeCameraModel::project3dToPixel outputs rectified pixel coordinates
    image_transport::CameraSubscriber sub_rgbImage = it.subscribeCamera(topic, 1, &CloudSegmentation::callback_rgbImage, &seg);
    ROS_INFO("Subscribing to %s", topic.c_str());


    ROS_INFO("----- Publisher -----");

    topic = "/segmented/points2";
    ros::Publisher pub_segmented_cloud = nh.advertise<CloudSegmentation::PointCloudT>(topic, 1);
    ROS_INFO("Publisher for segmented cloud set to %s", topic.c_str());

    topic = "/segmented/bounding_box_3d";
    ros::Publisher pub_box = nh.advertise<vision_msgs::Detection3D>(topic, 1);
    ROS_INFO("Publisher for objects set to %s", topic.c_str());

    topic = "/segmented/visualization_marker";
    ros::Publisher pub_viz_marker = nh.advertise<visualization_msgs::Marker>(topic, 1);
    ROS_INFO("Publisher for visualization marker set to %s", topic.c_str());

    topic = "/segmented/image_rect_color";
    image_transport::Publisher pub_rgb_image = it.advertise(topic, 1);
    ROS_INFO("Publisher for cropped image set to %s", topic.c_str());

    //ros::ServiceServer service = n.advertiseService("/hri_cloud_segmentation/Segment", &MinCutSegmentation::callback_gaze, dynamic_cast<MinCutSegmentation*>(&seg));

    ROS_INFO("---------------------");
    ROS_INFO("Waiting for initialization...");

    ros::Rate loop_rate(30);

    while (!seg.isInitialized()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("---------------------");
    ROS_INFO("Processing...");

    while (ros::ok())
    {
        seg.pass_through_filter(false);
        seg.voxel_filter(false);
        seg.planar_segmentation(30, false); // If epsilon angle equals 0 the axis is ignored.
        //seg.min_cut_segmentation(0.1, false);
        seg.clustering(false);
        seg.calc_bounding_box();
        seg.crop_image_to_bb();

        pub_segmented_cloud.publish(*seg.cloud_segmented);
        pub_box.publish(seg.object);
        pub_viz_marker.publish(seg.marker);
        pub_rgb_image.publish(seg.rgb_image_cropped_msg);

        ros::spinOnce();    // process callbacks
        loop_rate.sleep();
    }

    return (0);
}