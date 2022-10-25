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
        useSegmentation_(false),
        cloud_segmented_(new PointCloudT),
        isRecording_(false),
        isCamInfoSaved_(false),
        margin_factor_(1.0),
        data_path_(std::string(getenv("HOME")) + "/Pictures/object_data_v2"),
        rgb_cropped_folder_("rgb_cropped"),
        rgb_folder_("rgb"),
        depth_folder_("depth"),
        roi_folder_("roi"),
        tf_folder_("tf"),
        gaze_folder_("gaze")
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
    group_->setMaxVelocityScalingFactor(0.8);

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

    pub_received_cloud_ = nh_.advertise<PointCloudT>("/hri_robot_arm/received/points2", 1);

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

void ArmController::set_joint_constraints() {
    // Constraints because of camera
    moveit_msgs::Constraints path_constraints;
    moveit_msgs::OrientationConstraint ocm;
    ocm.header.frame_id = "j2n6s300_link_base";
    ocm.link_name = "j2n6s300_link_6";
    ocm.orientation.x = 0;
    ocm.orientation.y = -0.7071068;
    ocm.orientation.z = 0;
    ocm.orientation.w = 0.7071068;
    ocm.absolute_x_axis_tolerance = 3.14159 / 2;
    ocm.absolute_y_axis_tolerance = 3.14159 / 2;
    ocm.absolute_z_axis_tolerance = 2 * 3.14159;
    ocm.weight = 1.0;
    path_constraints.orientation_constraints.push_back(ocm);
    group_->setPathConstraints(path_constraints);

//    moveit_msgs::JointConstraint jcm1;
//    jcm1.joint_name = "j2n6s300_joint_1";
//    jcm1.position = 0;//current_state_.position.at(0);
//    ROS_INFO_STREAM(jcm1.position);
//    jcm1.tolerance_below = 3.14159 / 2;
//    jcm1.tolerance_above = 3.14159 / 2;
//    jcm1.weight = 1.0;
//    path_constraints.joint_constraints.push_back(jcm1);
//
//    moveit_msgs::JointConstraint jcm6;
//    jcm6.joint_name = "j2n6s300_joint_6";
//    jcm6.position = 0;//current_state_.position.at(5);
//    ROS_INFO_STREAM(jcm6.position);
//    jcm6.tolerance_below = 3.14159;
//    jcm6.tolerance_above = 3.14159;
//    jcm6.weight = 1.0;
//    path_constraints.joint_constraints.push_back(jcm6);

    group_->setPathConstraints(path_constraints);
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

    double correction_term = -0.05; // otherwise the object is a bit too far at the bottom of the image
    pose_msg.pose.position.x = targetpose_msg.position.x + delta_x;
    pose_msg.pose.position.y = targetpose_msg.position.y + delta_y;
    pose_msg.pose.position.z = targetpose_msg.position.z + delta_z; // + correction_term;
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
        while (!result && (count < 5))
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
            if ((pause_ == "r") || (pause_ == "R" ))
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
        if ((pause_ == "e") || (pause_ == "E"))
        {
            group.execute(my_plan_);
        }
    }
    ros::WallDuration(1.0).sleep();
}

void ArmController::record(const hri_robot_arm::RecordGoalConstPtr &goal)
{
    ROS_INFO_STREAM("Goal received for class \"" + goal->class_name + "\"");

    ROS_INFO_STREAM("Publish received cloud");
    pub_received_cloud_.publish(goal->segmented_cloud);

    // Set class name
    if (goal->class_name.empty())
    {
        result_.success = false;
        action_server_.setAborted(result_);
        ROS_INFO_STREAM("No class name provided. Abort.");
        return;
    }
    class_ = goal->class_name;

    if (useSegmentation_) {
        // Set segmented cloud
        pcl::fromROSMsg(goal->segmented_cloud, *cloud_segmented_);

        // Convert bounding box to root frame
        bbox_in_root_frame_ = goal->bbox;
        geometry_msgs::TransformStamped goal_to_root_frame = convert_bb_from_to(bbox_in_root_frame_, goal->header.frame_id, "root");

        // Convert gaze point to root frame
        tf2::doTransform(goal->gaze_point, gaze_point_.position, goal_to_root_frame);
    }
    else {
        // Convert gaze points to root frame
        geometry_msgs::TransformStamped goal_to_root_frame = get_transform_from_to(goal->header.frame_id, "root");
        std::vector<cv::Point3d> gaze_points_cv;
        std::vector<double> gaze_points_x, gaze_points_y, gaze_points_z;
        for (auto & gaze_point : goal->gaze_points) {
            geometry_msgs::Point gp_transformed;
            tf2::doTransform(gaze_point, gp_transformed, goal_to_root_frame);
            gaze_points_.push_back(gp_transformed);
            //gaze_points_cv.emplace_back(gp_transformed.x, gp_transformed.y, gp_transformed.z);
            gaze_points_x.emplace_back(gp_transformed.x);
            gaze_points_y.emplace_back(gp_transformed.y);
            gaze_points_z.emplace_back(gp_transformed.z);
        }
        // TODO use initial gaze point to filter (center +- 2*stdv) / as center

        // Calculate borders and center
//        cv::Point3d minPoint3D = get_minPoint(gaze_points_cv);
//        cv::Point3d maxPoint3D = get_maxPoint(gaze_points_cv);
//        cv::Point3d meanPoint3D = get_meanPoint(gaze_points_cv);

        auto const quantiles_x = calc_quantiles(gaze_points_x);
        auto const quantiles_y = calc_quantiles(gaze_points_y);
        auto const quantiles_z = calc_quantiles(gaze_points_z);

        // Approximate bounding box through gaze points
        bbox_in_root_frame_.center.position.x = quantiles_x.q50;
        bbox_in_root_frame_.center.position.y = quantiles_y.q50;
        bbox_in_root_frame_.center.position.z = quantiles_z.q50;
        // Values below q25-1.5*iqr and above q75+1.5*iqr are outliers
        // q75+1.5*iqr - (q25-1.5*iqr) = q75 - q25 + 3*iqr = 4*iqr
        bbox_in_root_frame_.size.x = 4 * quantiles_x.iqr;
        bbox_in_root_frame_.size.y = 4 * quantiles_y.iqr;
        bbox_in_root_frame_.size.z = 4 * quantiles_z.iqr;
    }


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
    if (readjust && useSegmentation_) {
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
//    std::vector<geometry_msgs::Pose> waypoints = calc_waypoints(gaze_point_, radius);

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
    group_->setMaxVelocityScalingFactor(0.5);
    update_directory();
    start_recording();
    group_->execute(trajectory); // Inspection/Evaluation does not work
    stop_recording();
    ROS_INFO_STREAM("Stop recording");
    group_->setMaxVelocityScalingFactor(0.8);
    gripper_group_->setNamedTarget("Close"); // Save space to not bump into anywhere
    gripper_group_->move();

    ROS_INFO_STREAM("Return to home position");
//    set_joint_constraints();
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
    if (pc_cropped->empty()) {
        ROS_INFO("Adjusted box invalid");
        return false;
    }
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

        // Check whether the pose is reachable
        group_->setPoseTarget(target_pose);
        moveit::core::MoveItErrorCode ret_code = group_->plan(my_plan_);
        bool result = (ret_code == moveit::core::MoveItErrorCode::SUCCESS);
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

double ArmController::calc_radius() const {
    double radius = 0.2; // Minimum distance to the object (realsense2 to end effector)
    double diag = std::sqrt(std::pow(bbox_in_root_frame_.size.x, 2)
                            + std::pow(bbox_in_root_frame_.size.y, 2)
                            + std::pow(bbox_in_root_frame_.size.z, 2));
    radius = diag > radius ? diag : radius;
    return 2.0 * radius;
}

void ArmController::callback_camera(const sensor_msgs::ImageConstPtr& img_msg,
                                    const sensor_msgs::ImageConstPtr& depth_msg,
                                    const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    if (isRecording_) {
        // Convert bounding box (also needed for tf files)
        vision_msgs::BoundingBox3D bbox_in_realsense_frame = bbox_in_root_frame_;
        geometry_msgs::TransformStamped tf_root_to_rs2 = convert_bb_from_to(bbox_in_realsense_frame,
                                                                            "root",
                                                                            "realsense2_color_optical_frame");

        // Points3D: corners of box resp. points of segmented cloud resp. gaze points
        std::vector<cv::Point3d> Points3D = std::vector<cv::Point3d>();

        // Images
        boost::shared_ptr<const cv_bridge::CvImage> rgb_image;
        boost::shared_ptr<const cv_bridge::CvImage> depth_image;

        // Needed if useSegmentation
        cv_bridge::CvImage rgb_image_cropped;
        cv::Point2d minPoint2D;
        cv::Point2d maxPoint2D;

        // Needed for gaze points
        std::vector<cv::Point2d> gaze_points_projected = std::vector<cv::Point2d>();

        if (useSegmentation_) {
            bool useBox = false;
            if (useBox) {
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

                Eigen::Vector3d Point3D;
                Point3D = position + orientation * (Eigen::Vector3d(size.x(), size.y(), size.z()) / 2);
                Points3D.emplace_back(Point3D.x(), Point3D.y(), Point3D.z());
                Point3D = position + orientation * (Eigen::Vector3d(-size.x(), size.y(), size.z()) / 2);
                Points3D.emplace_back(Point3D.x(), Point3D.y(), Point3D.z());
                Point3D = position + orientation * (Eigen::Vector3d(size.x(), -size.y(), size.z()) / 2);
                Points3D.emplace_back(Point3D.x(), Point3D.y(), Point3D.z());
                Point3D = position + orientation * (Eigen::Vector3d(size.x(), size.y(), -size.z()) / 2);
                Points3D.emplace_back(Point3D.x(), Point3D.y(), Point3D.z());
                Point3D = position + orientation * (Eigen::Vector3d(-size.x(), -size.y(), size.z()) / 2);
                Points3D.emplace_back(Point3D.x(), Point3D.y(), Point3D.z());
                Point3D = position + orientation * (Eigen::Vector3d(-size.x(), size.y(), -size.z()) / 2);
                Points3D.emplace_back(Point3D.x(), Point3D.y(), Point3D.z());
                Point3D = position + orientation * (Eigen::Vector3d(size.x(), -size.y(), -size.z()) / 2);
                Points3D.emplace_back(Point3D.x(), Point3D.y(), Point3D.z());
                Point3D = position + orientation * (Eigen::Vector3d(-size.x(), -size.y(), -size.z()) / 2);
                Points3D.emplace_back(Point3D.x(), Point3D.y(), Point3D.z());
            }
            else
            {
                // Get transform
                geometry_msgs::TransformStamped transform = get_transform_from_to(
                        cloud_segmented_->header.frame_id,
                        "realsense2_color_optical_frame");

                // Transform cloud from Azure Kinect Frame to RealSense Frame
                PointCloudT cloud_seg_transformed;
                pcl_ros::transformPointCloud(*cloud_segmented_, cloud_seg_transformed, transform.transform);

                // Convert to vector
                for (auto & point : cloud_seg_transformed.points) {
                    Points3D.emplace_back(point.x, point.y, point.z);
                }
            }

            // Project 3D corners/cloud points on 2D plane
            image_geometry::PinholeCameraModel cam_model;
            cam_model.fromCameraInfo(cam_info);
            std::vector<cv::Point2d> projected_corners = std::vector<cv::Point2d>();
            for (auto & point : Points3D) {
                cv::Point2d point2D = cam_model.project3dToPixel(point);
                projected_corners.push_back(point2D);
            }

            // Calculate 2D corners of box
            minPoint2D = get_minPoint(projected_corners);
            maxPoint2D = get_maxPoint(projected_corners);

            // Get rgb/depth image
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

            // Crop with extra space (margin)
            cv::Point2i center = cv::Point2i((int)(maxPoint2D.x + minPoint2D.x) / 2,
                                             (int)(maxPoint2D.y + minPoint2D.y) / 2);
            int width = (int)((maxPoint2D.x - minPoint2D.x) * margin_factor_);
            int height = (int)((maxPoint2D.y - minPoint2D.y) * margin_factor_);
            int x = (int)(center.x - width/2);
            int y = (int)(center.y - height/2);

            cv::Rect crop_region(x, y, width, height);
            crop_region &= cv::Rect(0, 0, img_msg->width, img_msg->height);
            if (crop_region.area() == 0) {
                ROS_INFO_STREAM("Cropping area is outside image dimensions. Image will not be saved.");
                return;
            }
            rgb_image_cropped = cv_bridge::CvImage(rgb_image->header, rgb_image->encoding);
            rgb_image->image(crop_region).copyTo(rgb_image_cropped.image);
        }
        else {
            // Get transform
            geometry_msgs::TransformStamped transform = get_transform_from_to(
                    "root",
                    "realsense2_color_optical_frame");

            // Transform gaze points from root frame to RealSense Frame
            for (auto & gaze_point : gaze_points_) {
                geometry_msgs::Point gp_transformed;
                tf2::doTransform(gaze_point, gp_transformed, transform);
                Points3D.emplace_back(gp_transformed.x, gp_transformed.y, gp_transformed.z);
            }

            // Project gaze points on 2D plane
            image_geometry::PinholeCameraModel cam_model;
            cam_model.fromCameraInfo(cam_info);
            for (auto & point : Points3D) {
                cv::Point2d point2D = cam_model.project3dToPixel(point);
                gaze_points_projected.push_back(point2D);
            }

            // Get rgb/depth image
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
        }

        // Save rgb and depth image to disk
        std::stringstream filename;
        cv::Mat bgr_image, bgr_image_cropped;
        filename << std::setw(4) << std::setfill('0') << img_counter_;
        std::string rgb_path_ = current_path_ + "/" + rgb_folder_ + "/" + filename.str() + "_rgb.jpg";
        std::string depth_path_ = current_path_ + "/" + depth_folder_ + "/" + filename.str() + "_depth.png";
        std::string rgb_cropped_path_ = current_path_ + "/" + rgb_cropped_folder_ + "/" + filename.str() + "_rgb_cropped.jpg";

        cv::cvtColor(rgb_image->image, bgr_image, cv::COLOR_RGB2BGR);
        cv::imwrite(rgb_path_, bgr_image);
        cv::imwrite(depth_path_, depth_image->image);
        if (useSegmentation_) {
            cv::cvtColor(rgb_image_cropped.image, bgr_image_cropped, cv::COLOR_RGB2BGR);
            cv::imwrite(rgb_cropped_path_, bgr_image_cropped);

            // Save ROI (without margin)
            YAML::Node yaml_node_roi;
            yaml_node_roi["ROI"]["x"] = (int) minPoint2D.x;
            yaml_node_roi["ROI"]["y"] = (int) minPoint2D.y;
            yaml_node_roi["ROI"]["width"] = (int)(maxPoint2D.x - minPoint2D.x);
            yaml_node_roi["ROI"]["height"] = (int)(maxPoint2D.y - minPoint2D.y);
            std::string tf_path_roi = current_path_ + "/" + roi_folder_ + "/" + filename.str() + "_roi.yaml";
            save_to_disk(tf_path_roi, yaml_node_roi);
        }
        else {
            // Save gaze points
            std::ostringstream gaze_ss;
            gaze_ss << "X; Y; Z" << std::endl;
            for (std::size_t i=0; i < gaze_points_projected.size(); ++i) {
                auto x = gaze_points_projected[i].x;
                auto y = gaze_points_projected[i].y;
                auto z = depth_image->image.at<unsigned short>(gaze_points_projected[i]);   // Points3D[i].z
                gaze_ss << x << ";" << y << ";" << z << std::endl;
            }
            std::string gaze_path = current_path_ + "/" + gaze_folder_ + "/" + filename.str() + "_gazepoints.csv";
            save_to_disk(gaze_path, gaze_ss.str());
        }

        // Calculate transformation
        tf2::Transform tf2_root_to_rs2, tf2_box_to_root, tf2_box_to_rs2;
        tf2::fromMsg(tf_root_to_rs2.transform, tf2_root_to_rs2);
        tf2::fromMsg(bbox_in_root_frame_.center, tf2_box_to_root);  // box_to_root maps (0,0,0)_box to (x,y,z)_root
        tf2_box_to_rs2.mult(tf2_root_to_rs2, tf2_box_to_root);
        geometry_msgs::Transform tf_rs2_to_box = tf2::toMsg(tf2_box_to_rs2.inverse());

        // Save camera-to-box-transformation to disk
        YAML::Node yaml_node_tf = YAML::Node(tf_rs2_to_box);
        std::string tf_path_ = current_path_ + "/" + tf_folder_ + "/" + filename.str() + "_tf.yaml";
        save_to_disk(tf_path_, yaml_node_tf);

        // Save camera info to disk (first time only)
        if (!isCamInfoSaved_) {
            YAML::Node yaml_node_cam_info = YAML::Node(*cam_info);
            save_to_disk(current_path_ + "/CameraInfo.yaml", yaml_node_cam_info);
            isCamInfoSaved_ = true;

            if (!useSegmentation_) {
                std::ostringstream gaze_ss;
                gaze_ss << "X; Y; Z" << std::endl;
                for (std::size_t i=0; i < gaze_points_.size(); ++i) {
                    auto x = gaze_points_[i].x;
                    auto y = gaze_points_[i].y;
                    auto z = gaze_points_[i].z;
                    gaze_ss << x << ";" << y << ";" << z << std::endl;
                }
                std::string gaze_path = current_path_ + "/gazepoints_root.csv";
                save_to_disk(gaze_path, gaze_ss.str());
            }
        }

        img_counter_++;
    }
}

template<class T>
ArmController::Quantiles ArmController::calc_quantiles(std::vector<T> vec) {
    // Note: Pseudo quantiles because if vector size is even, the real median is mean of the two values above and below

    auto const Q1 = vec.size() / 4;
    auto const Q2 = vec.size() / 2;
    auto const Q3 = Q1 + Q2;

    std::nth_element(vec.begin(),          vec.begin() + Q1, vec.end());
    std::nth_element(vec.begin() + Q1 + 1, vec.begin() + Q2, vec.end());
    std::nth_element(vec.begin() + Q2 + 1, vec.begin() + Q3, vec.end());

    Quantiles quantiles;
    quantiles.q25 = vec.at(Q1);
    quantiles.q50 = vec.at(Q2);
    quantiles.q75 = vec.at(Q3);
    quantiles.iqr = vec[Q3] - vec[Q1];

    return quantiles;
}

/// Pointwise (coordinates are independent from each other).
cv::Point2d ArmController::get_minPoint(const std::vector<cv::Point2d> &points) {
    double inf = std::numeric_limits<double>::infinity();
    cv::Point2d minPoint2D(inf, inf);
    for (auto & point : points)
    {
        minPoint2D.x = (point.x < minPoint2D.x) ? point.x : minPoint2D.x;
        minPoint2D.y = (point.y < minPoint2D.y) ? point.y : minPoint2D.y;
    }
    return minPoint2D;
}

/// Pointwise (coordinates are independent from each other).
cv::Point2d ArmController::get_maxPoint(const std::vector<cv::Point2d> &points) {
    double inf = std::numeric_limits<double>::infinity();
    cv::Point2d maxPoint2D(-inf, -inf);
    for (auto & point : points)
    {
        maxPoint2D.x = (point.x > maxPoint2D.x) ? point.x : maxPoint2D.x;
        maxPoint2D.y = (point.y > maxPoint2D.y) ? point.y : maxPoint2D.y;
    }
    return maxPoint2D;
}

/// Pointwise (coordinates are independent from each other).
cv::Point3d ArmController::get_minPoint(const std::vector<cv::Point3d> &points) {
    double inf = std::numeric_limits<double>::infinity();
    cv::Point3d minPoint3D(inf, inf, inf);
    for (auto & point : points)
    {
        minPoint3D.x = (point.x < minPoint3D.x) ? point.x : minPoint3D.x;
        minPoint3D.y = (point.y < minPoint3D.y) ? point.y : minPoint3D.y;
        minPoint3D.z = (point.z < minPoint3D.z) ? point.z : minPoint3D.z;
    }
    return minPoint3D;
}

/// Pointwise (coordinates are independent from each other).
cv::Point3d ArmController::get_maxPoint(const std::vector<cv::Point3d> &points) {
    double inf = std::numeric_limits<double>::infinity();
    cv::Point3d maxPoint3D(-inf, -inf, -inf);
    for (auto & point : points)
    {
        maxPoint3D.x = (point.x > maxPoint3D.x) ? point.x : maxPoint3D.x;
        maxPoint3D.y = (point.y > maxPoint3D.y) ? point.y : maxPoint3D.y;
        maxPoint3D.z = (point.z > maxPoint3D.z) ? point.z : maxPoint3D.z;
    }
    return maxPoint3D;
}

/// Pointwise (coordinates are independent from each other).
cv::Point3d ArmController::get_meanPoint(const std::vector<cv::Point3d> &points) {
    cv::Mat mat;
    cv::reduce(points, mat, 1, CV_REDUCE_AVG);  // mat has dimension 1xN_point with 3 channels
    cv::Point3d meanPoint3D(mat.at<double>(0,0), mat.at<double>(0,1), mat.at<double>(0,2));
    return  meanPoint3D;
}

void ArmController::update_directory() {
    current_path_ = data_path_ + "/" + class_;
    if (!boost::filesystem::exists(current_path_)) {
        current_path_.append("/" + class_ + "00");
    }
    else
    {
        std::string path(current_path_);
        for (int i = 0; boost::filesystem::exists(path) && (i < 100); ++i) {
            std::stringstream ss;
            ss << current_path_ << "/" << class_ << std::setw(2) << std::setfill('0') << i;
            path = ss.str();
        }
        current_path_ = path;
    }
    boost::filesystem::create_directories(current_path_);
    boost::filesystem::create_directories(current_path_ + "/" + rgb_folder_);
    boost::filesystem::create_directories(current_path_ + "/" + depth_folder_);
    boost::filesystem::create_directories(current_path_ + "/" + tf_folder_);
    if (useSegmentation_) {
        boost::filesystem::create_directories(current_path_ + "/" + rgb_cropped_folder_);
        boost::filesystem::create_directories(current_path_ + "/roi");
    }
    else {
        boost::filesystem::create_directories(current_path_ + "/" + gaze_folder_);
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
