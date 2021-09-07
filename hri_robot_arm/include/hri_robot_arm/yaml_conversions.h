//
// Created by weber on 07.09.21.
//

#ifndef HRI_ROBOT_ARM_YAML_CONVERSIONS_H
#define HRI_ROBOT_ARM_YAML_CONVERSIONS_H

#include "sensor_msgs/CameraInfo.h"
#include "geometry_msgs/Transform.h"


namespace YAML {


    template<>
    struct convert<geometry_msgs::Transform> {
        static Node encode(const geometry_msgs::Transform& tf) {
            Node node;
            node["translation"]["x"] = tf.translation.x;
            node["translation"]["y"] = tf.translation.y;
            node["translation"]["z"] = tf.translation.z;
            node["rotation"]["x"] = tf.rotation.x;
            node["rotation"]["y"] = tf.rotation.y;
            node["rotation"]["z"] = tf.rotation.z;
            node["rotation"]["w"] = tf.rotation.w;
            return node;
        }

        static bool decode(const Node& node, geometry_msgs::Transform& tf) {
            if(!node.IsMap() || node.size() != 2) {
                return false;
            }
            tf.translation.x = node["translation"]["x"].as<double>();
            tf.translation.y = node["translation"]["y"].as<double>();
            tf.translation.z = node["translation"]["z"].as<double>();
            tf.rotation.x = node["rotation"]["x"].as<double>();
            tf.rotation.y = node["rotation"]["y"].as<double>();
            tf.rotation.z = node["rotation"]["z"].as<double>();
            tf.rotation.w = node["rotation"]["w"].as<double>();
            return true;
        }
    };


    template<>
    struct convert<sensor_msgs::CameraInfo> {
        static Node encode(const sensor_msgs::CameraInfo& info) {
            Node node;
            node["header"]["seq"] = info.header.seq;
            node["header"]["stamp"]["sec"] = info.header.stamp.sec;
            node["header"]["stamp"]["nsec"] = info.header.stamp.nsec;
            node["header"]["frame_id"] = info.header.frame_id;
            node["height"] = info.height;
            node["width"] = info.width;
            node["distortion_model"] = info.distortion_model;
            for (auto & item : info.D) {
                node["D"].push_back(item);
            }
            for (auto & item : info.K) {
                node["K"].push_back(item);
            }
            for (auto & item : info.R) {
                node["R"].push_back(item);
            }
            for (auto & item : info.P) {
                node["P"].push_back(item);
            }
            node["binning_x"] = info.binning_x;
            node["binning_y"] = info.binning_y;
            node["roi"]["x_offset"] = info.roi.x_offset;
            node["roi"]["y_offset"] = info.roi.y_offset;
            node["roi"]["height"] = info.roi.height;
            node["roi"]["width"] = info.roi.width;
            node["roi"]["do_rectify"] = (bool)info.roi.do_rectify;
            return node;
        }

        static bool decode(const Node& node, sensor_msgs::CameraInfo& info) {
            if(!node.IsMap() || node.size() != 11) {
                return false;
            }
            info.header.seq = node["header"]["seq"].as<uint32_t>();
            info.header.stamp.sec = node["header"]["stamp"]["sec"].as<uint32_t>();
            info.header.stamp.nsec = node["header"]["stamp"]["nsec"].as<uint32_t>();
            info.header.frame_id = node["header"]["frame_id"].as<std::string>();
            info.height = node["height"].as<std::uint32_t>();
            info.width = node["width"].as<std::uint32_t>();
            info.distortion_model = node["distortion_model"].as<std::string>();
            for (int i = 0; i < node["D"].size(); i++) {
                info.D.at(i) = node["D"][i].as<double>();
            }
            for (int i = 0; i < node["K"].size(); i++) {
                info.K.at(i) = node["K"][i].as<double>();
            }
            for (int i = 0; i < node["R"].size(); i++) {
                info.R.at(i) = node["R"][i].as<double>();
            }
            for (int i = 0; i < node["P"].size(); i++) {
                info.P.at(i) = node["P"][i].as<double>();
            }
            info.binning_x = node["binning_x"].as<uint32_t>();
            info.binning_y = node["binning_y"].as<uint32_t>();
            info.roi.x_offset = node["roi"]["x_offset"].as<uint32_t>();
            info.roi.y_offset = node["roi"]["y_offset"].as<uint32_t>();
            info.roi.height = node["roi"]["height"].as<uint32_t>();
            info.roi.width = node["roi"]["width"].as<uint32_t>();
            info.roi.do_rectify = node["roi"]["do_rectify"].as<bool>();
            return true;
        }
    };
}

#endif //HRI_ROBOT_ARM_YAML_CONVERSIONS_H
