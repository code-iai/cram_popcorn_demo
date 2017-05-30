cmake_minimum_required(VERSION 2.8.3)
project(kinematics_msgs)

find_package(catkin REQUIRED
  message_generation
  actionlib_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  arm_navigation_msgs)

add_message_files(DIRECTORY msg FILES
  KinematicSolverInfo.msg  
  PositionIKRequest.msg)

add_service_files(DIRECTORY srv FILES
  GetConstraintAwarePositionIK.srv
  GetKinematicSolverInfo.srv
  GetPositionFK.srv
  GetPositionIK.srv)

generate_messages(DEPENDENCIES
  geometry_msgs
  actionlib_msgs
  sensor_msgs
  trajectory_msgs
  arm_navigation_msgs)

catkin_package(CATKIN_DEPENDS message_runtime) 
