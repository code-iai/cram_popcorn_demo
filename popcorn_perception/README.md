========= Steps to start the perceive_server ========= 

1. ssh erasmus@pr2a -> roslaunch /etc/ros/indigo/robot.launch
2. ssh erasmus@pr2b -> roslaunch /etc/ros/openni_head.launch
3. master-pr2a      -> roslaunch kitchen_context kitchen_context.launch
4. master-pr2a      -> roslaunch kitchen_context pr2_localization.launch
5. master-pr2a      -> roslaunch popcorn_perception perceive_action_server.launch 
6. master-pr2a      -> rostopic hz /kinect_head/depth_registered/points
7. master-pr2a      -> emacs -> ALT+X slime 
                             -> ,r-l-s popcorn_demo popcorn-demo 
                             -> (roslisp-utilities:startup-ros) 
                             -> (popcorn-demo::initialize-popcorn-demo)
                             -> (swank:operate-on-system-for-emacs "popcorn_perception-msg" (quote load-op))
                             -> (*)POT Example ---> (plan-library::detect-object kitchen-context::pot-designator) 



========= ROS commands ========= 
rosmsg show geometry_msgs/Pose
rostopic echo /perceive/result
rostopic echo /kinect_head/depth_registered/points
rosrun popcorn_perception perceive_server
rostopic pub /perceive_action/goal popcorn_perception/PerceiveActionGoal "goal:
  object_type: pot"



