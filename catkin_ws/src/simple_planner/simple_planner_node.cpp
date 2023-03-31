#include "simple_planner_node.h"

namespace {
    const double pre_grasp_offset = 0.05;
    const double gripper_open_command = 0.2;
    const double gripper_close_command = -0.2;
    const double gripper_tolerance = 0.05;

    void JointStateCallbackGripper(const sensor_msgs::JointState::ConstPtr &msg, double* gripper_pos)
    {
        // Look for the index of the "gripper_joint" in the message
        int idx = -1;
        for (size_t i = 0; i < msg->name.size(); i++)
        {
            if (msg->name[i] == "gripper_joint")
            {
                idx = i;
                break;
            }
        }

        if (idx >= 0)
        {
            // The gripper joint is present in the message, so print its position
            *gripper_pos = msg->position[idx];
        }
        else
        {
            // The gripper joint is not present in the message
            ROS_WARN("Gripper joint not found in JointState message");
        }
    }
}

void SimplePlannerNode::Initialize()
{
    gripper_subscriber_ = subscribe<sensor_msgs::JointState>(
        "/joint_states", 10, boost::bind(JointStateCallbackGripper, _1, &gripper_value_));
    move_group_.setPlanningTime(2.5);
    InitializeCollisions();
}

void SimplePlannerNode::MoveToHomePose()
{
    move_group_.setJointValueTarget(move_group_.getNamedTargetValues("home"));
    moveit::planning_interface::MoveGroupInterface::Plan moveit_plan;
    if(move_group_.plan(moveit_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_WARN("Cannot move to home position");
    }
    move_group_.move();
    OpenGripper();
}

void SimplePlannerNode::Pick(const geometry_msgs::Pose target_pose)
{
    geometry_msgs::Pose target_pose_above = target_pose;
    target_pose_above.position.z += pre_grasp_offset;

    TryMove(intermediate_pose_right_);
    TryMove(intermediate_pose_left_);
    TryMove(target_pose_above);
    TryMove(target_pose);
    CloseGripper();
    TryMove(target_pose_above);
}

void SimplePlannerNode::Place(const geometry_msgs::Pose target_pose)
{
    geometry_msgs::Pose target_pose_above = target_pose;
    target_pose_above.position.z += pre_grasp_offset;

    TryMove(intermediate_pose_left_);
    TryMove(intermediate_pose_right_);
    TryMove(target_pose_above);
    TryMove(target_pose);
    OpenGripper();
    TryMove(target_pose_above);
}

geometry_msgs::Pose SimplePlannerNode::GetCurrentEefPose()
{
    return move_group_.getCurrentPose("ee_link").pose;
}

void SimplePlannerNode::SetOrientationConstraintsToCurrentOrientation()
{
    moveit_msgs::OrientationConstraint ori_constraint;
    ori_constraint.link_name = "ee_link";
    ori_constraint.header.frame_id = move_group_.getPoseReferenceFrame();
    ori_constraint.orientation.x = GetCurrentEefPose().orientation.x;
    ori_constraint.orientation.y = GetCurrentEefPose().orientation.y;;
    ori_constraint.orientation.z = GetCurrentEefPose().orientation.z;;
    ori_constraint.orientation.w = GetCurrentEefPose().orientation.w;;
    ori_constraint.absolute_x_axis_tolerance = M_PI/6;
    ori_constraint.absolute_y_axis_tolerance = M_PI/6;
    ori_constraint.absolute_z_axis_tolerance = M_PI/6;
    ori_constraint.weight = 1.0;

    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.emplace_back(ori_constraint);

    move_group_.setPathConstraints(constraints);
}

void SimplePlannerNode::OpenGripper()
{
    std_msgs::Float64 gripper_msg;
    gripper_msg.data = gripper_open_command;

    while (gripper_value_ < gripper_open_command - gripper_tolerance)
    {
        gripper_command_publisher_.publish(gripper_msg);
        ros::WallDuration(0.05).sleep();
    }
}

void SimplePlannerNode::CloseGripper()
{
    std_msgs::Float64 gripper_msg;
    gripper_msg.data = gripper_close_command;

    while (gripper_value_ > gripper_close_command + gripper_tolerance)
    {
        gripper_command_publisher_.publish(gripper_msg);
        ros::WallDuration(0.05).sleep();
    }
}

void SimplePlannerNode::TryMove(const geometry_msgs::Pose& target_pose)
{

    move_group_.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan moveit_plan;
    if(move_group_.plan(moveit_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_WARN("Trajectory not foud, moving a little and replanning");
        geometry_msgs::Pose intermediate_pose = target_pose;
        intermediate_pose.position.x = (GetCurrentEefPose().position.x + target_pose.position.x)/2.0;
        intermediate_pose.position.y = (GetCurrentEefPose().position.y + target_pose.position.y)/2.0;
        intermediate_pose.position.z = (GetCurrentEefPose().position.z + target_pose.position.z)/2.0;
        TryMove(intermediate_pose);  // hack to prevent moveit from getting stuck not finding a plan
        TryMove(target_pose);
    }

    move_group_.execute(moveit_plan);
}

void SimplePlannerNode::InitializeCollisions()
{
  // Add the column
  collision_objects_.emplace_back();
  collision_objects_[0].id = "column";
  collision_objects_[0].header.frame_id = "base_link_inertia";

  collision_objects_[0].primitives.resize(1);
  collision_objects_[0].primitives[0].type = collision_objects_[0].primitives[0].BOX;
  collision_objects_[0].primitives[0].dimensions.resize(3);
  collision_objects_[0].primitives[0].dimensions[0] = 0.042;
  collision_objects_[0].primitives[0].dimensions[1] = 0.042;
  collision_objects_[0].primitives[0].dimensions[2] = 0.74;

  collision_objects_[0].primitive_poses.resize(1);
  collision_objects_[0].primitive_poses[0].position.x = 0;
  collision_objects_[0].primitive_poses[0].position.y = 0;
  collision_objects_[0].primitive_poses[0].position.z = -0.33;

  collision_objects_[0].operation = collision_objects_[0].ADD;

  // Add the left table
  moveit_msgs::CollisionObject table;
  table.id = "cafe_table_1";
  table.header.frame_id = "base_link_inertia";

  table.primitives.resize(1);
  table.primitives[0].type = collision_objects_[0].primitives[0].BOX;
  table.primitives[0].dimensions.resize(3);
  table.primitives[0].dimensions[0] = 0.913;
  table.primitives[0].dimensions[1] = 0.913;
  table.primitives[0].dimensions[2] = 0.04;

  table.primitive_poses.resize(1);
  table.primitive_poses[0].position.x = 0.0;
  table.primitive_poses[0].position.y = -0.6;
  table.primitive_poses[0].position.z = 0.055;
  
  table.operation = collision_objects_[1].ADD;

  collision_objects_.push_back(table);

  // Add the right table
  table.id = "cafe_table_2";
  table.header.frame_id = "base_link_inertia";
  table.primitive_poses[0].position.y = 0.6; 
  collision_objects_.push_back(std::move(table));

  planning_scene_interface_.applyCollisionObjects(collision_objects_);
}
