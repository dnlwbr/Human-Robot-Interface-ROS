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
        nh_(nh),
        tf_listener_(tf_buffer_),
        action_server_("/hri_robot_arm/Record", boost::bind(&ArmController::record, this, _1), false),
        isRecording_(false),
        isCamInfoSaved_(false),
        crop_factor_(0.8),
        data_path_(std::string(getenv("HOME")) + "/Pictures/object_data"),
        rgb_folder_("rgb"),
        depth_folder_("depth"),
        tf_folder_("tf")
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

    group_->setPoseReferenceFrame("root");
//    group_->setEndEffectorLink(robot_type_ + "_end_effector");
    group_->setEndEffectorLink("realsense2_end_effector");
    group_->setMaxVelocityScalingFactor(0.5);

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

    action_server_.start();
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
    co_.primitive_poses[0].position.z = -0.03/2.0 - 0.12;
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
    co_.primitives.resize(2);
    co_.primitive_poses.resize(2);
    co_.primitives[1].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[1].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[1].dimensions[shape_msgs::SolidPrimitive::BOX_X] = bbox_in_root_frame_.size.x;
    co_.primitives[1].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = bbox_in_root_frame_.size.y;
    co_.primitives[1].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = bbox_in_root_frame_.size.z;

    co_.primitive_poses[1].position = bbox_in_root_frame_.center.position;
    co_.primitive_poses[1].orientation = bbox_in_root_frame_.center.orientation;

    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
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
geometry_msgs::PoseStamped ArmController::generate_gripper_align_pose(const geometry_msgs::Pose& targetpose_msg, double dist, double azimuth, double polar, double rot_gripper_z)
{
    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.frame_id = "root";
    pose_msg.header.stamp = ros::Time::now();

    // computer pregrasp position w.r.t. location of grasp_pose in spherical coordinate. Orientation is w.r.t. fixed world (root) reference frame.
    double delta_x = -dist * cos(azimuth) * sin(polar);
    double delta_y = -dist * sin(azimuth) * sin(polar);
    double delta_z = -dist * cos(polar);

    // computer the orientation of gripper w.r.t. fixed world (root) reference frame. The gripper (z axis) should point(open) to the grasp_pose.
    tf::Quaternion q = EulerZYZ_to_Quaternion(azimuth, polar, rot_gripper_z);

    pose_msg.pose.position.x = targetpose_msg.position.x + delta_x;
    pose_msg.pose.position.y = targetpose_msg.position.y + delta_y;
    pose_msg.pose.position.z = targetpose_msg.position.z + delta_z;
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": " << "pose_msg: x " << pose_msg.pose.position.x  << ", y " << pose_msg.pose.position.y  << ", z " << pose_msg.pose.position.z  << ", qx " << pose_msg.pose.orientation.x  << ", qy " << pose_msg.pose.orientation.y  << ", qz " << pose_msg.pose.orientation.z  << ", qw " << pose_msg.pose.orientation.w );

    return pose_msg;
}

void ArmController::evaluate_plan(moveit::planning_interface::MoveGroupInterface &group)
{
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

void ArmController::record(const hri_robot_arm::RecordGoalConstPtr &goal)
{
    ROS_INFO_STREAM("Goal received for class \"" + goal->class_name + "\"");

    // Set class name
    if (goal->class_name.empty())
    {
        result_.success = false;
        action_server_.setAborted(result_);
        ROS_INFO_STREAM("No class name provided. Abort.");
        return;
    }
    class_ = goal->class_name;

    // Convert bounding box to root frame
    bbox_in_root_frame_ = goal->bbox;
    convert_bb_from_to(bbox_in_root_frame_, goal->header.frame_id, "root");

    ROS_INFO_STREAM("Build workspace");
    clear_workscene();
    ros::WallDuration(0.5).sleep();
    build_workscene();
    ros::WallDuration(0.5).sleep();
    group_->clearPathConstraints();

    ROS_INFO_STREAM("Send robot to home position");
    group_->setNamedTarget("Home");
    group_->move();

    ROS_INFO("Add bounding box as obstacle");
    add_obstacle();
    ros::WallDuration(0.5).sleep();

    bool readjust = false;
    if (readjust) {
        ROS_INFO("Calculate inspect position");
        double radius = calc_radius();
        geometry_msgs::Pose inspect_pose = calc_inspect_pose(bbox_in_root_frame_.center, radius);

        ROS_INFO("Go to inspect position");
        group_->setPoseTarget(inspect_pose);
        group_->move();

        ROS_INFO("Readjust bounding box");
        bool ret = readjust_box();
        if (!ret) {
            ROS_INFO("Readjusting failed - Return to home position");
            group_->setNamedTarget("Home");
            group_->move();
            clear_workscene();
            result_.success = false;
            action_server_.setAborted(result_);
            ROS_INFO_STREAM("Finished");
            ROS_INFO_STREAM("Waiting for new goal...");
            return;
        }

        ROS_INFO("Update obstacle");
        add_obstacle();
        ros::WallDuration(0.5).sleep();
    }

    ROS_INFO("Calculate waypoints");
    double radius = calc_radius();
    std::vector<geometry_msgs::Pose> waypoints = calc_waypoints(bbox_in_root_frame_.center, radius);

    ROS_INFO_STREAM("Go to start position");
    group_->setPoseTarget(waypoints.front());
    group_->move(); // evaluate_plan(*group_);

    ROS_INFO("Compute path");
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group_->computeCartesianPath(waypoints, 0.01,5.0, trajectory);
    ROS_INFO("Visualizing cartesian path (%.2f%% of the reachable waypoints included)",  fraction * 100.0);

    ROS_INFO_STREAM("Start recording...");
    gripper_group_->setNamedTarget("Open"); // Move the fingers out of the field of view of the camera
    gripper_group_->move();
    ros::WallDuration(0.5).sleep();
    update_directory();
    start_recording();
    group_->execute(trajectory); // Inspection/Evaluation does not work
    stop_recording();
    ROS_INFO_STREAM("Stop recording");
    gripper_group_->setNamedTarget("Close"); // Save space to not bump into anywhere
    gripper_group_->move();

    ROS_INFO_STREAM("Return to home position");
    group_->setNamedTarget("Home");
    group_->move();

//  If need to double check if reach target position.
//    replacement of group_->getCurrentPose();
//    { // scope for mutex update
//        boost::mutex::scoped_lock lock_state(mutex_state_);
//        geometry_msgs::PoseStamped copy_pose = current_pose_;
//    }

    clear_workscene();
    ros::WallDuration(0.5).sleep();
    result_.success = percentage_reachable_ > 60;
    if (result_.success == true)
    {
        action_server_.setSucceeded(result_);
    }
    else
    {
        action_server_.setAborted(result_);
    }
    ROS_INFO_STREAM("Finished");
    ROS_INFO_STREAM("Waiting for new goal...");
}

/**
 * @brief Converts a bounding box between two frames and additionally returns used transformation.
 * @param[in,out] box Inplace conversion of box from source_frame to target_frame
 * @param[in] source_frame The frame where the data originated
 * @param[in] target_frame The frame to which data should be transformed
 * @return Transfrom from source_frame to target_frame
 */
geometry_msgs::TransformStamped ArmController::convert_bb_from_to(vision_msgs::BoundingBox3D &box,
                                       const std::string& source_frame,
                                       const std::string& target_frame)
{
    // Get transform
    geometry_msgs::TransformStamped transform = get_transform_from_to(source_frame, target_frame);

    // Transform center pose
    tf2::doTransform(box.center, box.center, transform);

    return transform;
}

geometry_msgs::TransformStamped ArmController::get_transform_from_to(const std::string& source_frame,
                                                                     const std::string& target_frame)
{
    geometry_msgs::TransformStamped transform;
    try{
        transform = tf_buffer_.lookupTransform(target_frame,
                                               source_frame,
                                               ros::Time(0),
                                               ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Failure: %s", ex.what());
    }
    return transform;
}

bool ArmController::readjust_box() {
    // Get point cloud
    std::string topic = "/realsense2/depth/color/points";
    PointCloudT::ConstPtr pc = ros::topic::waitForMessage<PointCloudT>(topic, ros::Duration(20));
    if (pc == nullptr) {
        ROS_WARN("No point cloud messages received");
        return false;
    }
    ROS_INFO("Point cloud received");

    // Crop pointcloud to existing bounding box
    PointCloudT::Ptr pc_cropped(new PointCloudT);
    crop_cloud_to_bb(pc,pc_cropped);
    if (pc_cropped->empty())
        ROS_INFO("Adjusted box invalid");
        return false;
    geometry_msgs::TransformStamped transform = get_transform_from_to("realsense2_color_optical_frame", "root");
    sensor_msgs::PointCloud2 pc_cropped_in_root_frame_msg;
    pcl::toROSMsg(*pc_cropped, pc_cropped_in_root_frame_msg);
    tf2::doTransform(pc_cropped_in_root_frame_msg, pc_cropped_in_root_frame_msg, transform);
    pcl::fromROSMsg(pc_cropped_in_root_frame_msg, *pc_cropped);

    // Get min and max
    PointT minPoint3D, maxPoint3D;
    pcl::getMinMax3D(*pc_cropped, minPoint3D, maxPoint3D);

    // Calculate 3D bounding box
    bbox_in_root_frame_.size.x = maxPoint3D.x - minPoint3D.x;
    bbox_in_root_frame_.size.y = maxPoint3D.y - minPoint3D.y;
    bbox_in_root_frame_.size.z = maxPoint3D.z - minPoint3D.z;
    bbox_in_root_frame_.center.position.x = (minPoint3D.x + maxPoint3D.x) / 2;
    bbox_in_root_frame_.center.position.y = (minPoint3D.y + maxPoint3D.y) / 2;
    bbox_in_root_frame_.center.position.z = (minPoint3D.z + maxPoint3D.z) / 2;
    bbox_in_root_frame_.center.orientation.x = 0;
    bbox_in_root_frame_.center.orientation.y = 0;
    bbox_in_root_frame_.center.orientation.z = 0;
    bbox_in_root_frame_.center.orientation.w = 1;

    return true;
}

void ArmController::crop_cloud_to_bb(const PointCloudT::ConstPtr& pc_in, PointCloudT::Ptr& pc_out) {
    // Convert bounding box
    vision_msgs::BoundingBox3D bbox_in_realsense_frame = bbox_in_root_frame_;
    convert_bb_from_to(bbox_in_realsense_frame, "root", "realsense2_color_optical_frame");

    // Calculate min & max
    Eigen::Vector3f position = Eigen::Vector3f((float)bbox_in_realsense_frame.center.position.x,
                                               (float)bbox_in_realsense_frame.center.position.y,
                                               (float)bbox_in_realsense_frame.center.position.z);
    Eigen::Quaternionf orientation = Eigen::Quaternionf((float)bbox_in_realsense_frame.center.orientation.w,
                                                        (float)bbox_in_realsense_frame.center.orientation.x,
                                                        (float)bbox_in_realsense_frame.center.orientation.y,
                                                        (float)bbox_in_realsense_frame.center.orientation.z);
    Eigen::Vector3f size = Eigen::Vector3f((float)bbox_in_realsense_frame.size.x,
                                           (float)bbox_in_realsense_frame.size.y,
                                           (float)bbox_in_realsense_frame.size.z);

    Eigen::Vector3f minPoint = -size / 2;
    Eigen::Vector3f maxPoint = size / 2;

    // Crop box
    pcl::CropBox<PointT> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minPoint.x(), minPoint.y(), minPoint.z(), 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxPoint.x(), maxPoint.y(), maxPoint.z(), 1.0));
    boxFilter.setTranslation(position);
    auto euler = orientation.toRotationMatrix().eulerAngles(0,1,2);
    boxFilter.setRotation(euler);

    boxFilter.setInputCloud(pc_in);
    boxFilter.filter(*pc_out);
}

geometry_msgs::Pose ArmController::calc_inspect_pose(const geometry_msgs::Pose& center, double radius)
{
    geometry_msgs::PoseStamped inspect_pose;
    double angle_resolution_deg = 10;
    double diff_angle_rad = angle_resolution_deg * 3.14/180;
    double angle_rad = 0;
    bool reachable = false;
    while (!reachable) {
        angle_rad -= diff_angle_rad; // clockwise rotation ("+" for counterclockwise)
        inspect_pose = generate_gripper_align_pose(center, radius, angle_rad, 3*M_PI/4, M_PI/2);
        // Check whether the pose is reachable
        group_->setPoseTarget(inspect_pose);
        reachable = (group_->plan(my_plan_) == moveit_msgs::MoveItErrorCodes::SUCCESS);
        group_->clearPoseTargets();
    }
    return inspect_pose.pose;
}

std::vector<geometry_msgs::Pose> ArmController::calc_waypoints(const geometry_msgs::Pose& center, double radius)
{
    // Create Cartesian Paths
    std::vector<geometry_msgs::Pose> waypoints; // end_effector_trajectory
    std::vector<geometry_msgs::Pose> waypoints2; // second part, if unreachable points in between
    geometry_msgs::PoseStamped target_pose;

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
        target_pose = generate_gripper_align_pose(center, radius, angle_rad, 3*M_PI/4, M_PI/2);

        // Adjust target_pose
        //target_pose.pose.position.z -= 0.05; // TODO?

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
        feedback_.progress = 100.0 * i / (360/angle_resolution_deg);
        action_server_.publishFeedback(feedback_);
    }
    feedback_.progress = 100.0;
    action_server_.publishFeedback(feedback_);

    // Concatenate the valid waypoints
    waypoints.insert(waypoints.begin(), waypoints2.begin(), waypoints2.end());
    percentage_reachable_ = 100.0 * (float)waypoints.size() / (360/angle_resolution_deg);
    ROS_INFO("%zu of the %.0f waypoints are reachable (%.2f%%)", waypoints.size(), 360/angle_resolution_deg, percentage_reachable_);

    // Reset planning time to previous value
    group_->setPlanningTime(planning_time);

    return waypoints;
}

double ArmController::calc_radius() {
    double radius = 0.2; // Minimum distance to the object (realsense2 to end effector)
    radius = (bbox_in_root_frame_.size.x) > radius ? bbox_in_root_frame_.size.x : radius;
    radius = (bbox_in_root_frame_.size.y) > radius ? bbox_in_root_frame_.size.y : radius;
    radius = (bbox_in_root_frame_.size.z) > radius ? bbox_in_root_frame_.size.z : radius;
    return 3.0 * radius;
}

void ArmController::callback_camera(const sensor_msgs::ImageConstPtr& img_msg,
                                    const sensor_msgs::ImageConstPtr& depth_msg,
                                    const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    if (isRecording_) {
        // Convert bounding box
        vision_msgs::BoundingBox3D bbox_in_realsense_frame = bbox_in_root_frame_;
        geometry_msgs::TransformStamped tf_root_to_rs2 = convert_bb_from_to(bbox_in_realsense_frame,
                                                                            "root",
                                                                            "realsense2_color_optical_frame");

        // Calculate 3D corners
        Eigen::Vector3d position = Eigen::Vector3d(bbox_in_realsense_frame.center.position.x,
                                                   bbox_in_realsense_frame.center.position.y,
                                                   bbox_in_realsense_frame.center.position.z);
        Eigen::Quaterniond orientation = Eigen::Quaterniond(bbox_in_realsense_frame.center.orientation.w,
                                                            bbox_in_realsense_frame.center.orientation.x,
                                                            bbox_in_realsense_frame.center.orientation.y,
                                                            bbox_in_realsense_frame.center.orientation.z);
        Eigen::Vector3d size = Eigen::Vector3d(bbox_in_realsense_frame.size.x,
                                               bbox_in_realsense_frame.size.y,
                                               bbox_in_realsense_frame.size.z);

        std::vector<cv::Point3d> corners = std::vector<cv::Point3d>();
        Eigen::Vector3d Point3D;
        Point3D = position + orientation * (Eigen::Vector3d(size.x(), size.y(), size.z()) / 2);
        corners.emplace_back(Point3D.x(), Point3D.y(), Point3D.z());
        Point3D = position + orientation * (Eigen::Vector3d(-size.x(), size.y(), size.z()) / 2);
        corners.emplace_back(Point3D.x(), Point3D.y(), Point3D.z());
        Point3D = position + orientation * (Eigen::Vector3d(size.x(), -size.y(), size.z()) / 2);
        corners.emplace_back(Point3D.x(), Point3D.y(), Point3D.z());
        Point3D = position + orientation * (Eigen::Vector3d(size.x(), size.y(), -size.z()) / 2);
        corners.emplace_back(Point3D.x(), Point3D.y(), Point3D.z());
        Point3D = position + orientation * (Eigen::Vector3d(-size.x(), -size.y(), size.z()) / 2);
        corners.emplace_back(Point3D.x(), Point3D.y(), Point3D.z());
        Point3D = position + orientation * (Eigen::Vector3d(-size.x(), size.y(), -size.z()) / 2);
        corners.emplace_back(Point3D.x(), Point3D.y(), Point3D.z());
        Point3D = position + orientation * (Eigen::Vector3d(size.x(), -size.y(), -size.z()) / 2);
        corners.emplace_back(Point3D.x(), Point3D.y(), Point3D.z());
        Point3D = position + orientation * (Eigen::Vector3d(-size.x(), -size.y(), -size.z()) / 2);
        corners.emplace_back(Point3D.x(), Point3D.y(), Point3D.z());

        // Project 3D corners on 2D plane
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(cam_info);
        std::vector<cv::Point2d> projected_corners = std::vector<cv::Point2d>();
        for (auto & corner : corners) {
            cv::Point2d point2D = cam_model.project3dToPixel(corner);
            projected_corners.push_back(point2D);
        }

        // Calculate 2D corners of box
        double inf = std::numeric_limits<double>::infinity();
        cv::Point2d minPoint2D(inf, inf);
        cv::Point2d maxPoint2D(-inf, -inf);
        for (auto & point : projected_corners)
        {
            minPoint2D.x = (point.x < minPoint2D.x) ? point.x : minPoint2D.x;
            minPoint2D.y = (point.y < minPoint2D.y) ? point.y : minPoint2D.y;
            maxPoint2D.x = (point.x > maxPoint2D.x) ? point.x : maxPoint2D.x;
            maxPoint2D.y = (point.y > maxPoint2D.y) ? point.y : maxPoint2D.y;
        }
        minPoint2D.x = (minPoint2D.x < 0) ? 0 : minPoint2D.x;
        minPoint2D.y = (minPoint2D.y < 0) ? 0 : minPoint2D.y;
        maxPoint2D.x = (maxPoint2D.x >= img_msg->width) ? img_msg->width - 1 : maxPoint2D.x;
        maxPoint2D.y = (maxPoint2D.y >= img_msg->height) ? img_msg->height - 1 : maxPoint2D.y;

        // Crop image to 2D bounding box
        boost::shared_ptr<const cv_bridge::CvImage> rgb_image;
        boost::shared_ptr<const cv_bridge::CvImage> depth_image;
        try
        {
            // Convert message to CvImage
            rgb_image = cv_bridge::toCvShare(img_msg);
            depth_image = cv_bridge::toCvShare(depth_msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        double margin_factor = 1.3;
        cv::Point2i center = cv::Point2i((int)(maxPoint2D.x + minPoint2D.x) / 2,
                                         (int)(maxPoint2D.y + minPoint2D.y) / 2);
        int width = (int)((maxPoint2D.x - minPoint2D.x) * margin_factor);
        int height = (int)((maxPoint2D.y - minPoint2D.y) * margin_factor);
        int x = (int)(center.x - width/2);
        int y = (int)(center.y - height/2);
/*
        cv::Point2d center = cv::Point2d(img_msg->width/2.0,img_msg->height/2.0);
        auto width = (int)(img_msg->width * crop_factor_);
        auto height = (int)(img_msg->height * crop_factor_);
        auto x = (int)(center.x - (img_msg->width * crop_factor_)/2.0);
        auto y = (int)(center.y - (img_msg->height * crop_factor_)/2.0);
*/
        cv::Rect crop_region(x, y, width, height);
        cv_bridge::CvImage rgb_image_cropped = cv_bridge::CvImage(rgb_image->header, rgb_image->encoding);
        cv_bridge::CvImage depth_image_cropped = cv_bridge::CvImage(depth_image->header, depth_image->encoding);
        rgb_image->image(crop_region).copyTo(rgb_image_cropped.image);
        depth_image->image(crop_region).copyTo(depth_image_cropped.image);

        // Save rgb and depth image to disk
        std::stringstream filename;
        filename << std::setw(3) << std::setfill('0') << img_counter_;
        std::string rgb_path_ = current_path_ + "/" + rgb_folder_ + "/" + filename.str() + "_rgb.jpg";
        std::string depth_path_ = current_path_ + "/" + depth_folder_ + "/" + filename.str() + "_depth.png";
        cv::imwrite(rgb_path_, rgb_image_cropped.image);
        cv::imwrite(depth_path_, depth_image_cropped.image);

        // Calculate transformation
        tf2::Transform tf2_root_to_rs2, tf2_box_to_root, tf2_box_to_rs2;
        tf2::fromMsg(tf_root_to_rs2.transform, tf2_root_to_rs2);
        tf2::fromMsg(bbox_in_root_frame_.center, tf2_box_to_root);  // box_to_root maps (0,0,0)_box to (x,y,z)_root
        tf2_box_to_rs2.mult(tf2_root_to_rs2, tf2_box_to_root);
        geometry_msgs::Transform tf_rs2_to_box = tf2::toMsg(tf2_box_to_rs2.inverse());

        // Save camera-to-box-transformation to disk
        YAML::Node yaml_node = YAML::Node(tf_rs2_to_box);
        std::string tf_path_ = current_path_ + "/" + tf_folder_ + "/" + filename.str() + "_tf.yaml";
        save_to_disk(tf_path_, yaml_node);

        // Save camera info to disk (first time only)
        if (!isCamInfoSaved_) {
            sensor_msgs::CameraInfo cam_info_crop = *cam_info;
            if (crop_factor_ != 1) {
                // Adjust the camera intrinsics to the cropped images
                cam_info_crop.height = height;
                cam_info_crop.width = width;
                cam_info_crop.K.elems[2] = cam_info_crop.K.elems[2] - (img_msg->width - width) / 2.0;
                cam_info_crop.K.elems[5] = cam_info_crop.K.elems[5] - (img_msg->height - height) / 2.0;
                cam_info_crop.P.elems[2] = cam_info_crop.K.elems[2];
                cam_info_crop.P.elems[6] = cam_info_crop.K.elems[5];
            }
            yaml_node = YAML::Node(cam_info_crop);
            save_to_disk(current_path_ + "/CameraInfo.yaml", yaml_node);
            isCamInfoSaved_ = true;
        }

        img_counter_++;
    }
}

void ArmController::update_directory() {
    current_path_ = data_path_ + "/" + class_;
    if (!boost::filesystem::exists(current_path_)) {
        current_path_.append("/" + class_ + "00");
        boost::filesystem::create_directories(current_path_);
        boost::filesystem::create_directories(current_path_ + "/" + rgb_folder_);
        boost::filesystem::create_directories(current_path_ + "/" + depth_folder_);
        boost::filesystem::create_directories(current_path_ + "/" + tf_folder_);
    }
    else
    {
        std::string path(current_path_);
        for (int i = 0; boost::filesystem::exists(path) && i < 100; ++i) {
            std::stringstream ss;
            ss << current_path_ << "/" << class_ << std::setw(2) << std::setfill('0') << i;
            path = ss.str();
        }
        current_path_ = path;
        boost::filesystem::create_directories(current_path_);
        boost::filesystem::create_directories(current_path_ + "/" + rgb_folder_);
        boost::filesystem::create_directories(current_path_ + "/" + depth_folder_);
        boost::filesystem::create_directories(current_path_ + "/" + tf_folder_);
    }
    ROS_INFO_STREAM("Output path: " + current_path_);
}

template<class T>
void ArmController::save_to_disk(const std::string& path, T data) {
    fstream_out_.open(path);
    if(!fstream_out_) {
        ROS_INFO_STREAM("Error: \"" + path + "\" could not be opened");
    }
    fstream_out_ << data;
    fstream_out_.close();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "hri_robot_arm");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    hri_arm::ArmController arm_controller(node);

    // Note: The realsense2 streams are already rectified internally by the firmware
    message_filters::Subscriber<sensor_msgs::Image> image_sub(node, "/realsense2/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(node, "/realsense2/aligned_depth_to_color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(node, "/realsense2/color/camera_info", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> sync(image_sub, depth_sub, info_sub, 10);
    sync.registerCallback(boost::bind(&ArmController::callback_camera, &arm_controller, _1, _2, _3));

    ros::waitForShutdown();
    return 0;
}
