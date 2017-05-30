======== abstract plans testing steps ========


1. roslaunch kitchen_context kitchen_context_simulation.launch
2. lauch pr2 localization
- ssh erasmus@pr2a
- export ROS_MASTER_URI=http://lisca-desktop:11311 or export ROS_MASTER_URI=http://chemlab-simulator:11311
- roslaunch kitchen_context pr2_localization.launch
3. start CRAM
- emacs
- M+x RET
- slime RET
- ,
- ros-load-system RET
- popcorn-demo RET
- (popcorn-demo::execute-tests)


======= How to get PR2's frame =========
1. Example to find a grasping frame for the deep plate
(tf-utilities::lookup-current-frame-relative-to-refference-frame "l_gripper_tool_frame" "deep_plate_frame")
