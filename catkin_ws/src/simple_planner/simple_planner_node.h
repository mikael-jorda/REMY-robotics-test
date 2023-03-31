#ifndef SIMPLE_PLANNER_NODE_H
#define SIMPLE_PLANNER_NODE_H

#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/GetModelState.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

class SimplePlannerNode : public ros::NodeHandle {
public:
    SimplePlannerNode() = default;
    ~SimplePlannerNode() = default;

    void Initialize();

    // move to home joint positions
    void MoveToHomePose();

    // pick a block at the given position.
    // moves through the right intermediate pose and left intermediate pose,
    // then on top of the block, goes down, grasps the block and goes back up
    // using a sequence of plan and move actions from moveit
    void Pick(const geometry_msgs::Pose target_pose);

    // place a block at the given position.
    // moves through the left intermediate pose and right intermediate pose,
    // then on top of the desired block position, goes down, releases the block and goes back up
    // using a sequence of plan and move actions from moveit
    void Place(const geometry_msgs::Pose target_pose);
    
    // returns moveit current eef pose
    geometry_msgs::Pose GetCurrentEefPose();

    // sets the left and right waypoints positions
    void SetIntermediatePose(const geometry_msgs::Pose pose) {
        intermediate_pose_left_ = pose;
        intermediate_pose_right_ = pose;
        intermediate_pose_left_.position.y -= 0.35;
    }

    // sets the orientation constraints to match the current orientation
    void SetOrientationConstraintsToCurrentOrientation();

private:

    // sends an open or close message to the grasper controller
    // until it is detected that the grasper was opened or closed
    void OpenGripper();
    void CloseGripper();

    // computes a plan to move to the target pose
    // and executes it if the planning is successful
    // is not successful, tries to move to the middle point first
    void TryMove(const geometry_msgs::Pose& target_pose);

    // initializes the collisions
    void InitializeCollisions();

    // moveit interfaces
    moveit::planning_interface::MoveGroupInterface::Plan moveit_plan_;
    moveit::planning_interface::MoveGroupInterface move_group_ {"manipulator"};
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_ {};
    std::vector<moveit_msgs::CollisionObject> collision_objects_{};

    // ros publishers and subscribers
    ros::Subscriber gripper_subscriber_;
    ros::Publisher gripper_command_publisher_ = advertise<std_msgs::Float64>("/gripper_joint_position/command", 10);

    // other member variables
    double gripper_value_;
    geometry_msgs::Pose intermediate_pose_left_;
    geometry_msgs::Pose intermediate_pose_right_;

};

#endif // SIMPLE_PLANNER_NODE_H