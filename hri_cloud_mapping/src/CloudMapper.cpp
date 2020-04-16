//
// Created by weber on 06.04.20.
//

#include "../include/hri_cloud_mapping/CloudMapper.h"
#include <iostream>

void CloudMapper::callback_human(PointCloudT::ConstPtr const & msg) {
    human_cloud = msg;
    isInitializedHC = true;
    std::cout << "Callback human" << std::endl;
}

void CloudMapper::callback_robot(PointCloudT::ConstPtr const & msg) {
    robot_cloud = msg;
    isInitializedRC = true;
}

void ICPMapper::setInput() {
    // Iterative Closest Point algorithm
    icp.setInputSource(human_cloud);
    icp.setInputTarget(robot_cloud);

}


//void NDTMapper::setInput() {}
//void NDTMapper::align() {}