cmake_minimum_required(VERSION 3.0.2)
project(raisim_teleop)

list(APPEND CMAKE_PREFIX_PATH )

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(Boost REQUIRED COMPONENTS system)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  lipm_msgs
  actionlib
  nav_msgs
  tf
)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES step_planner2d_ros
  CATKIN_DEPENDS roscpp rospy sensor_msgs tf std_msgs geometry_msgs nav_msgs lipm_msgs actionlib
  DEPENDS system_lib EIGEN3
)


include_directories (include ${EIGEN3_INCLUDE_DIRS}  ${catkin_INCLUDE_DIRS} )


add_executable(atlas_raisim_teleop src/atlas_raisim_teleop.cpp  src/AtlasStepPlanner2D.cpp src/AtlasRobotParameters.cpp )
target_link_libraries(atlas_raisim_teleop ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

add_executable(nao_raisim_teleop src/nao_raisim_teleop.cpp  src/NaoStepPlanner2D.cpp src/NaoRobotParameters.cpp )
target_link_libraries(nao_raisim_teleop ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
