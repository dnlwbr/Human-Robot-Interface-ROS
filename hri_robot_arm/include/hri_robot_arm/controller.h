#ifndef HRI_ROBOT_ARM_CONTROLLER_H
#define HRI_ROBOT_ARM_CONTROLLER_H

#include <ros/ros.h>
#include <kinova_driver/kinova_ros_types.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <kinova_msgs/SetFingersPositionAction.h>

#include <vision_msgs/BoundingBox3D.h>
#include "hri_robot_arm/RecordAction.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <moveit/kinematic_constraints/utils.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>

// Camera
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <boost/filesystem.hpp>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "hri_robot_arm/yaml_conversions.h"


namespace hri_arm
{


    class ArmController
    {
    public:
        explicit ArmController(ros::NodeHandle &nh);
        ~ArmController();
        void callback_camera(const sensor_msgs::ImageConstPtr& rgb_image,
                             const sensor_msgs::ImageConstPtr& depth_image,
                             const sensor_msgs::CameraInfoConstPtr& cam_info);


    private:
        ros::NodeHandle nh_;

        // open&close fingers: gripper_group_.plan not alway have a solution
        actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>* finger_client_;

        moveit::planning_interface::MoveGroupInterface* group_;
        moveit::planning_interface::MoveGroupInterface* gripper_group_;
        robot_model::RobotModelPtr robot_model_;
//        robot_state::RobotStatePtr robot_state_;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_;

        planning_scene::PlanningScenePtr planning_scene_;
        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

        // work scene
        moveit_msgs::CollisionObject co_;
        moveit_msgs::PlanningScene planning_scene_msg_;

        ros::Publisher pub_co_;
        ros::Publisher pub_planning_scene_diff_;
        ros::Subscriber sub_pose_;
        ros::Subscriber sub_joint_;
        actionlib::SimpleActionServer<hri_robot_arm::RecordAction> action_server_;

        // create messages that are used to published feedback/result
        hri_robot_arm::RecordFeedback feedback_;
        hri_robot_arm::RecordResult result_;
        double percentage_reachable_{};

        //
        std::vector<std::string> joint_names_;
        std::vector<double> joint_values_;

        // wait for user input to continue: cin >> pause_;
        std::string pause_;
        std::string robot_type_;
        bool robot_connected_{};

        // update current state and pose
        boost::mutex mutex_state_;
        boost::mutex mutex_pose_;
        sensor_msgs::JointState current_state_;
        geometry_msgs::PoseStamped current_pose_;

        // Transformation
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // Output file stream to write to disk
        std::ofstream fstream_out_;

        // Bounding box
        vision_msgs::BoundingBox3D bbox_in_root_frame_;

        // Recordings
        std::string data_path_;
        std::string current_path_;
        std::string class_;
        bool isRecording_;
        bool isCamInfoSaved_;
        unsigned int img_counter_{};
        double crop_factor_;

        void get_current_state(const sensor_msgs::JointStateConstPtr &msg);
        void get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg);

        void clear_workscene();
        void build_workscene();
        void clear_obstacle();
        void add_obstacle();

        static geometry_msgs::PoseStamped generate_gripper_align_pose(const geometry_msgs::Pose& targetpose_msg, double dist, double azimuth, double polar, double rot_gripper_z);
        void evaluate_plan(moveit::planning_interface::MoveGroupInterface &group);
        void record(const hri_robot_arm::RecordGoalConstPtr &goal);
        geometry_msgs::TransformStamped get_transform_from_to(const std::string& source_frame, const std::string& target_frame);
        geometry_msgs::TransformStamped convert_bb_from_to(vision_msgs::BoundingBox3D &box, const std::string& source_frame, const std::string& target_frame);
        std::vector<geometry_msgs::Pose> calc_waypoints(const geometry_msgs::Pose& center, double radius);
        double calc_radius();

        inline void start_recording() { img_counter_ = 0; isRecording_ = true; isCamInfoSaved_ = false;}
        inline void stop_recording() { isRecording_ = false;}
        void update_directory();
        template<class T> void save_to_disk(const std::string& path, T data);
    };
}


#endif // HRI_ROBOT_ARM_CONTROLLER_H
