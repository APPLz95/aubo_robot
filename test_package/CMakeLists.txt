cmake_minimum_required(VERSION 3.0.2)
project(test_package)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf
  tf2_ros
  #tf2_eigen
  tf2_geometry_msgs
)


catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS
  scripts/simple_publisher.py
  scripts/simple_subscriber.py
  scripts/MoveGroupInterfacePython.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Declare a C++ executable
## With catkin_make MoveGroupInterface_To_Kinetic package is built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(MoveGroupInterface scripts/MoveGroupInterface.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(MoveGroupInterface ${catkin_LIBRARIES} ${Boost_LIBRARIES})