cmake_minimum_required(VERSION 3.0.2)
project(simple_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  moveit_ros_planning_interface
  gazebo_msgs
  geometry_msgs
)

include_directories(
  ${Boost_INCLUDE_DIRS}  
  ${catkin_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIRS}
)

catkin_package()

add_executable(${PROJECT_NAME} main.cpp simple_planner_node.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
