=== Gazebo-based Popcorn Demo ===

1. update your ~/.bashrc 
   export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/your_workspace/catkin_ws/src/kitchen_context/models

2. launch popcorn world
   roslaunch kitchen_context kitchen_context_simulation.launch 

3. lauch pr2 localization
   * ssh erasmus@pr2a
   * export ROS_MASTER_URI=http://lisca-desktop:11311 or export ROS_MASTER_URI=http://chemlab-simulator:11311
   * roslaunch kitchen_context pr2_localization.launch

4. start CRAM
   * emacs
   * M+x RET
   * slime RET
   * ,
   * ros-load-system RET
   * popcorn-demo RET
   * (roslisp-utilities:startupros)
   * (popcorn-demo::initialize-popcorn-demo)


=== example to query the semantic map ===


--------------Console--------------
1. start json_prolog
   roslaunch kitchen_context knowrob.launch

2. start server
   rosrun tf2_ros buffer_server 

 

--------------EMACS--------------
2. start Slime
   M-X

3. load package
   (swank:operate-on-system-for-emacs "kitchen-context" (quote load-op))

4. start a ros node
   (roslisp-utilities:startup-ros)

5. enter simple semantic map query
   (kitchen-context::query-semantic-map 
          "current_object_pose"
          "http://knowrob.org/kb/stove_table_semantic_map.owl#StoveTable_RbTLNllQ")


