#include "simple_planner_node.h"

int main(int argc, char** argv)
{

  // Initialize ROS
  ros::init(argc, argv, "simple_planner");

  // create and initialize the planner_node
  SimplePlannerNode planner_node;
  planner_node.Initialize();

  // start spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // move to home pose and set orientation constraints
  planner_node.MoveToHomePose();
  planner_node.SetOrientationConstraintsToCurrentOrientation();

  // record home pose
  geometry_msgs::Pose eef_home_pose = planner_node.GetCurrentEefPose();
  // set intermediate pose to planner
  geometry_msgs::Pose intermediate_pose = eef_home_pose;
  intermediate_pose.position.z -= 0.10;
  intermediate_pose.position.x += 0.10;
  planner_node.SetIntermediatePose(intermediate_pose);

  // Get the block initial positions
  const int n_blocks = 5;
  const std::string block_prefix = "block_";
  std::vector<geometry_msgs::Pose> blocks_poses;
  // blocks_poses.resize(5);
  ros::ServiceClient block_pose_client = planner_node.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  gazebo_msgs::GetModelState srv;
  for (int i=0; i<n_blocks; ++i){
    srv.request.model_name = block_prefix + std::to_string(i+1);
    while (!block_pose_client.call(srv))
    {
      ros::WallDuration(0.5).sleep();
    };
    blocks_poses.push_back(std::move(srv.response.pose));
  }

  // initialize pick and place poses for orientation
  const double gripper_z_offset = 0.2;
  const double robot_z_in_world = 0.7;  // offset between the simulation world (blocks pos reference) and the robot world in Z direction
  geometry_msgs::Pose place_pose = eef_home_pose;
  place_pose.position.x = 0.35;
  place_pose.position.y = 0.65;
  place_pose.position.z = blocks_poses[0].position.z - robot_z_in_world;

  geometry_msgs::Pose pick_pose = eef_home_pose;

  for (int i = 0; i < n_blocks; ++i)
  {
    // find desired pose for next block pick
    pick_pose.position.x = blocks_poses[i].position.x;
    pick_pose.position.y = blocks_poses[i].position.y;
    pick_pose.position.z = blocks_poses[i].position.z - robot_z_in_world;

    // find desired pose for next block place, place them on a line
    place_pose.position.y -= 0.08;

    // pick and place
    planner_node.Pick(pick_pose);
    planner_node.Place(place_pose);
  }

  // move to home pose
  planner_node.MoveToHomePose();

  // exit
  ros::shutdown();
  return 0;
}