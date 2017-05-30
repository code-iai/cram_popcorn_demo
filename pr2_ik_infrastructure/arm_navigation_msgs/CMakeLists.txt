cmake_minimum_required(VERSION 2.8.3)
project(arm_navigation_msgs)

find_package(catkin REQUIRED COMPONENTS 
             message_generation 
             actionlib_msgs
             std_msgs 
             geometry_msgs
             sensor_msgs
             trajectory_msgs
             tf)

add_message_files(
  DIRECTORY msg
  FILES
  AllowedCollisionEntry.msg        MotionPlanRequest.msg
  AllowedCollisionMatrix.msg       MoveArmStatistics.msg
  AllowedContactSpecification.msg  MultiDOFJointState.msg
  ArmNavigationErrorCodes.msg      MultiDOFJointTrajectory.msg
  AttachedCollisionObject.msg      MultiDOFJointTrajectoryPoint.msg
  CollisionMap.msg                 OrderedCollisionOperations.msg
  CollisionObject.msg              OrientationConstraint.msg
  CollisionObjectOperation.msg     OrientedBoundingBox.msg
  CollisionOperation.msg           PlanningScene.msg
  Constraints.msg                  PositionConstraint.msg
  ContactInformation.msg           RobotState.msg
  DisplayTrajectory.msg            RobotTrajectory.msg
  JointConstraint.msg              Shape.msg
  JointLimits.msg                  SimplePoseConstraint.msg
  JointTrajectoryWithLimits.msg    VisibilityConstraint.msg
  LinkPadding.msg                  WorkspaceParameters.msg)

add_service_files(DIRECTORY srv FILES
  FilterJointTrajectory.srv                 GetPlanningScene.srv
  FilterJointTrajectoryWithConstraints.srv  GetRobotState.srv
  GetCollisionObjects.srv                   GetRobotTrajectoryValidity.srv
  GetJointTrajectoryValidity.srv            GetStateValidity.srv
  GetMotionPlan.srv                         SetPlanningSceneDiff.srv)

add_action_files(DIRECTORY action FILES
  MakeStaticCollisionMap.action  
  MoveArm.action  
  SyncPlanningScene.action)

generate_messages(DEPENDENCIES 
                  actionlib_msgs
                  std_msgs 
                  geometry_msgs
                  sensor_msgs
                  trajectory_msgs
                  )

catkin_package(CATKIN_DEPENDS 
               message_runtime)

#install(DIRECTORY rules
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
