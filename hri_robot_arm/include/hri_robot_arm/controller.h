#ifndef HRI_ROBOT_ARM_CONTROLLER_H
#define HRI_ROBOT_ARM_CONTROLLER_H

#include <ros/ros.h>
#include <kinova_driver/kinova_ros_types.h>

#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>

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

namespace hri_arm
{


    class ArmController
    {
    public:
        ArmController(ros::NodeHandle &nh);
        ~ArmController();



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

        //
        std::vector<std::string> joint_names_;
        std::vector<double> joint_values_;

        // check some process if success.
        bool result_;
        // wait for user input to continue: cin >> pause_;
        std::string pause_;
        std::string robot_type_;
        bool robot_connected_{};

        // update current state and pose
        boost::mutex mutex_state_;
        boost::mutex mutex_pose_;
        sensor_msgs::JointState current_state_;
        geometry_msgs::PoseStamped current_pose_;


        geometry_msgs::PoseStamped center_;

        void build_workscene();
        void add_obstacle();
        void clear_obstacle();
        void clear_workscene();

        void define_cartesian_pose();
        static geometry_msgs::PoseStamped generate_gripper_align_pose(const geometry_msgs::PoseStamped& targetpose_msg, double dist, double azimuth, double polar, double rot_gripper_z);

        bool circular_movement();
        std::vector<geometry_msgs::Pose> calc_waypoints(const geometry_msgs::PoseStamped& center, double radius);

        void get_current_state(const sensor_msgs::JointStateConstPtr &msg);
        void get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg);
        void evaluate_plan(moveit::planning_interface::MoveGroupInterface &group, bool inspect_first = true);
    };
}


#endif // HRI_ROBOT_ARM_CONTROLLER_H
