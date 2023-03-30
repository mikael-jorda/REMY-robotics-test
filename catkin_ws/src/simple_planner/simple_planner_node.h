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

    void MoveToHomePose();

    void Pick(const geometry_msgs::Pose target_pose);
    void Place(const geometry_msgs::Pose target_pose);
    
    geometry_msgs::Pose GetCurrentEefPose();

    void SetIntermediatePose(const geometry_msgs::Pose pose) {
        intermediate_pose_left_ = pose;
        intermediate_pose_right_ = pose;
        intermediate_pose_left_.position.y -= 0.35;
    }

    void SetOrientationConstraintsToCurrentOrientation();

private:

    void OpenGripper();
    void CloseGripper();
    void TryMove(const geometry_msgs::Pose& target_pose);

    void InitializeCollisions();

    moveit::planning_interface::MoveGroupInterface::Plan moveit_plan_;
    moveit::planning_interface::MoveGroupInterface move_group_ {"manipulator"};
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_ {};
    std::vector<moveit_msgs::CollisionObject> collision_objects_{};

    ros::Subscriber gripper_subscriber_;
    ros::Publisher gripper_command_publisher_ = advertise<std_msgs::Float64>("/gripper_joint_position/command", 10);

    double gripper_value_;
    geometry_msgs::Pose intermediate_pose_left_;
    geometry_msgs::Pose intermediate_pose_right_;

};

#endif // SIMPLE_PLANNER_NODE_H