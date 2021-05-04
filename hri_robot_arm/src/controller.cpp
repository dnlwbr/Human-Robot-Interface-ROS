#include "../include/hri_robot_arm/controller.h"
#include <ros/console.h>

#include <tf_conversions/tf_eigen.h>


using namespace hri_arm;


tf::Quaternion EulerZYZ_to_Quaternion(double tz1, double ty, double tz2)
{
    tf::Quaternion q;
    tf::Matrix3x3 rot;
    tf::Matrix3x3 rot_temp;
    rot.setIdentity();

    rot_temp.setEulerYPR(tz1, 0.0, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(0.0, ty, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(tz2, 0.0, 0.0);
    rot *= rot_temp;
    rot.getRotation(q);
    return q;
}


ArmController::ArmController(ros::NodeHandle &nh):
    nh_(nh)
{
    ros::NodeHandle pn("~");

    nh_.param<std::string>("/robot_type",robot_type_,"j2n6s300");
    nh_.param<bool>("/robot_connected",robot_connected_,true);

    if (robot_connected_)
    {
        sub_joint_ = nh_.subscribe<sensor_msgs::JointState>("/" + robot_type_ + "_driver/out/joint_state", 1, &ArmController::get_current_state, this);
        sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/" + robot_type_ +"_driver/out/tool_pose", 1, &ArmController::get_current_pose, this);
    }

    // Before we can load the planner, we need two objects, a RobotModel and a PlanningScene.
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();

    // construct a `PlanningScene` that maintains the state of the world (including the robot).
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

//    //  every time need retrive current robot state, do the following.
//    robot_state::RobotState& robot_state = planning_scene_->getCurrentStateNonConst();
//    const robot_state::JointModelGroup *joint_model_group = robot_state.getJointModelGroup("arm");

    group_ = new moveit::planning_interface::MoveGroupInterface("arm");
    gripper_group_ = new moveit::planning_interface::MoveGroupInterface("gripper");

    group_->setEndEffectorLink(robot_type_ + "_end_effector");

    finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
            ("/" + robot_type_ + "_driver/fingers_action/finger_positions", false);
    while(robot_connected_ && !finger_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the finger action server to come up");
    }

    pub_co_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
    pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    int arm_joint_num = robot_type_[3]-'0';
    joint_names_.resize(arm_joint_num);
    joint_values_.resize(joint_names_.size());
    for (uint i = 0; i<joint_names_.size(); i++)
    {
        joint_names_[i] = robot_type_ + "_joint_" + boost::lexical_cast<std::string>(i+1);
    }

    // set pre-defined joint and pose values.
    define_cartesian_pose();


    circular_movement();
}


ArmController::~ArmController()
{
    // shut down pub and subs
    sub_joint_.shutdown();
    sub_pose_.shutdown();
    pub_co_.shutdown();
    pub_planning_scene_diff_.shutdown();

    // release memory
    delete group_;
    delete gripper_group_;
    delete finger_client_;
}


void ArmController::get_current_state(const sensor_msgs::JointStateConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_state_);
    current_state_ = *msg;
}

void ArmController::get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_pose_);
    current_pose_ = *msg;
}

void ArmController::clear_workscene()
{
    // remove table
    co_.id = "table";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    // remove target
    co_.id = "target_cylinder";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    planning_scene_msg_.world.collision_objects.clear();
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);

    clear_obstacle();
}

void ArmController::build_workscene()
{
    co_.header.frame_id = "root";
    co_.header.stamp = ros::Time::now();

    // remove table
    co_.id = "table";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    // add table
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2.4;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.4;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.03;
    co_.primitive_poses[0].position.x = 0;
    co_.primitive_poses[0].position.y = 0.0;
    co_.primitive_poses[0].position.z = -0.03/2.0;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();}

void ArmController::clear_obstacle()
{
    co_.id = "box";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
    //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": remove pole ");
    //      std::cin >> pause_;
}

void ArmController::add_obstacle()
{
    clear_obstacle();

    co_.id = "box";
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.1;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.1;

    co_.primitive_poses[0].position = center_.pose.position;

    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
}

void ArmController::define_cartesian_pose()
{
    tf::Quaternion q;

    // define start pose before grasp
    center_.header.frame_id = "root";
    center_.header.stamp = ros::Time::now();
    center_.pose.position.x = 0.0;
    center_.pose.position.y = -0.5;
    center_.pose.position.z = 0.1;

    q = EulerZYZ_to_Quaternion(M_PI/4, M_PI/2, M_PI);
    center_.pose.orientation.x = q.x();
    center_.pose.orientation.y = q.y();
    center_.pose.orientation.z = q.z();
    center_.pose.orientation.w = q.w();
}

/**
 * @brief ArmController::generate_gripper_align_pose
 * @param targetpose_msg pick/place pose (object location): where gripper close/open the fingers (grasp/release the object). Only position information is used.
 * @param dist distance of returned pose to targetpose
 * @param azimuth an angle measured from the x-axis in the xy-plane in spherical coordinates, denoted theta (0<= theta < 2pi ).
 * @param polar also named zenith, colatitude, denoted phi (0<=phi<=pi). It is the angle from the positive z-axis to the vector.  phi= pi/2 - delta where delta is the latitude.
 * @param rot_gripper_z rotation along the z axis of the gripper reference frame (last joint rotation)
 * @return a pose defined in a spherical coordinates where origin is located at the target pose. Normally it is a pre_grasp/post_realease pose, where gripper axis (last joint axis) is pointing to the object (target_pose).
 */
geometry_msgs::PoseStamped ArmController::generate_gripper_align_pose(const geometry_msgs::PoseStamped& targetpose_msg, double dist, double azimuth, double polar, double rot_gripper_z)
{
    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.frame_id = "root";

    // computer pregrasp position w.r.t. location of grasp_pose in spherical coordinate. Orientation is w.r.t. fixed world (root) reference frame.
    double delta_x = -dist * cos(azimuth) * sin(polar);
    double delta_y = -dist * sin(azimuth) * sin(polar);
    double delta_z = -dist * cos(polar);

    // computer the orientation of gripper w.r.t. fixed world (root) reference frame. The gripper (z axis) should point(open) to the grasp_pose.
    tf::Quaternion q = EulerZYZ_to_Quaternion(azimuth, polar, rot_gripper_z);

    pose_msg.pose.position.x = targetpose_msg.pose.position.x + delta_x;
    pose_msg.pose.position.y = targetpose_msg.pose.position.y + delta_y;
    pose_msg.pose.position.z = targetpose_msg.pose.position.z + delta_z;
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": " << "pose_msg: x " << pose_msg.pose.position.x  << ", y " << pose_msg.pose.position.y  << ", z " << pose_msg.pose.position.z  << ", qx " << pose_msg.pose.orientation.x  << ", qy " << pose_msg.pose.orientation.y  << ", qz " << pose_msg.pose.orientation.z  << ", qw " << pose_msg.pose.orientation.w );

    return pose_msg;

}

void ArmController::evaluate_plan(moveit::planning_interface::MoveGroupInterface &group, bool inspect_first)
{
    if (!inspect_first) {
        group.move();
        return;
    }

    bool result = false;
    bool replan = true;
    int count = 0;

    while (replan && ros::ok())
    {
        // reset flag for replan
        count = 0;
        result = false;

        // try to find a success plan.
        double plan_time;
        while (!result && count < 5)
        {
            count++;
            plan_time = 20+count*10;
            ROS_INFO("Setting plan time to %f sec", plan_time);
            group.setPlanningTime(plan_time);
            result = (group.plan(my_plan_) == moveit_msgs::MoveItErrorCodes::SUCCESS);
            std::cout << "at attemp: " << count << std::endl;
            ros::WallDuration(0.1).sleep();
        }

        // found a plan
        if (result)
        {
            std::cout << "plan success at attemp: " << count << std::endl;

            replan = false;
            std::cout << "please input e to execute the plan, r to replan, others to skip: ";
            std::cin >> pause_;
            ros::WallDuration(0.5).sleep();
            if (pause_ == "r" || pause_ == "R" )
            {
                replan = true;
            }
            else
            {
                replan = false;
            }
        }
        else // not found
        {
            std::cout << "Exit since plan failed until reach maximum attemp: " << count << std::endl;
            replan = false;
            break;
        }
    }

    if(result)
    {
        if (pause_ == "e" || pause_ == "E")
        {
            group.execute(my_plan_);
        }
    }
    ros::WallDuration(1.0).sleep();
}

bool ArmController::circular_movement()
{
    bool inspect_first = false;
    //group_->setMaxVelocityScalingFactor(0.1);

    clear_workscene();
    ros::WallDuration(1.0).sleep();
    build_workscene();
    ros::WallDuration(1.0).sleep();
    group_->clearPathConstraints();

    ROS_INFO_STREAM("Send robot to home position ...");
    group_->setNamedTarget("Home");
    evaluate_plan(*group_, inspect_first);
    ros::WallDuration(1.0).sleep();
    gripper_group_->setNamedTarget("Open");
    gripper_group_->move();

    ROS_INFO("Add bounding box as obstacle");
    add_obstacle(); // TODO Size
    ros::WallDuration(1.0).sleep();

    ROS_INFO("Calculate waypoints ...");
    std::vector<geometry_msgs::Pose> waypoints = calc_waypoints(center_, 0.15);

    ROS_INFO_STREAM("Go to start position  ...");
    group_->setPoseTarget(waypoints.front());
    evaluate_plan(*group_, inspect_first);

    ROS_INFO("Compute path ...");
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group_->computeCartesianPath(waypoints, 0.01,5.0, trajectory);
//    ROS_INFO("Visualizing cartesian path (%.2f%% achieved)",  fraction * 100.0);

    ROS_INFO_STREAM("Start recording ...");
    my_plan_.trajectory_ = trajectory;
    group_->execute(trajectory); // Inspection/Evaluation does not work

    ROS_INFO_STREAM("Return to home position  ...");
    group_->setNamedTarget("Home");
    evaluate_plan(*group_, inspect_first);

//  If need to double check if reach target position.
//    replacement of group_->getCurrentPose();
//    { // scope for mutex update
//        boost::mutex::scoped_lock lock_state(mutex_state_);
//        geometry_msgs::PoseStamped copy_pose = current_pose_;
//    }

    clear_workscene();
    ros::WallDuration(1.0).sleep();
    return true;
}

std::vector<geometry_msgs::Pose> ArmController::calc_waypoints(const geometry_msgs::PoseStamped& center, double radius)
{
    // Create Cartesian Paths
    std::vector<geometry_msgs::Pose> waypoints; // end_effector_trajectory
    std::vector<geometry_msgs::Pose> waypoints2; // second part, if unreachable points in between
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "root";

    // Trajectory parameters (circle)
    double angle_resolution_deg = 10;
    double diff_angle_rad = angle_resolution_deg * 3.14/180;
    double angle_rad = 0;

    // Plan for the trajectory
    // (temporarily reduce planning time due to the delay caused by failed attempts)
    double planning_time = group_->getPlanningTime();
    group_->setPlanningTime(0.25);
    bool gap = false;
    for (int i = 0; i< (360/angle_resolution_deg); i++)
    {
        // Discretize the trajectory
        angle_rad -= diff_angle_rad; // clockwise rotation ("+" for counterclockwise)
        target_pose = generate_gripper_align_pose(center_, radius, angle_rad, 3*M_PI/4, M_PI/2);
        target_pose.header.stamp = ros::Time::now();

        // Check whether the pose is reachable
        group_->setPoseTarget(target_pose);
        bool result = (group_->plan(my_plan_) == moveit_msgs::MoveItErrorCodes::SUCCESS);
        group_->clearPoseTargets();
        if (result) {
            if (!gap) {
                waypoints.push_back(target_pose.pose);
            }
            else {
                waypoints2.push_back(target_pose.pose);
            }
        }
        else {
            gap = true;
        }
    }
    // Concatinate the valid waypoints
    waypoints.insert(waypoints.begin(), waypoints2.begin(), waypoints2.end());
    double percent = 100.0 * (float)waypoints.size() / (360/angle_resolution_deg);
    ROS_INFO("%.2f%% of the path is reachable.", percent);
    //    ROS_INFO("There are %zu number of waypoints", waypoints.size());

    // Reset planning time to previous value
    group_->setPlanningTime(planning_time);

    return waypoints;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "hri_arm");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    hri_arm::ArmController arm_controller(node);

    ros::spin();
    return 0;
}
