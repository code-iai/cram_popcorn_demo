/*
 * ik_trajectory_executor.cpp
 *
 *  Created on: 06.02.2011
 *      Author: keiser
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>
//#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <two_hand_ik_trajectory_executor/ExecuteRightArmCartesianIKTrajectory.h>
#include <two_hand_ik_trajectory_executor/ExecuteLeftArmCartesianIKTrajectory.h>
#include <two_hand_ik_trajectory_executor/ExecuteBothArmsCartesianIKTrajectory.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <map>
#include <string>

#define MAX_JOINT_VEL 0.5  //in radians/sec
static const std::string RIGHT_ARM_IK_NAME = "/pr2_right_arm_kinematics/get_ik";
static const std::string LEFT_ARM_IK_NAME = "/pr2_left_arm_kinematics/get_ik";
static const std::string RIGHT_ARM_FK_NAME = "/pr2_right_arm_kinematics/get_fk";
static const std::string LEFT_ARM_FK_NAME = "/pr2_left_arm_kinematics/get_fk";
//static const std::string RIGHT_ARM_FK_INFO = "/pr2_right_arm_kinematics/get_fk_solver_info";
//static const std::string LEFT_ARM_FK_INFO = "/pr2_left_arm_kinematics/get_fk_solver_info";
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

//static const bool splitCartesianTrajectories = false;
//static const double maxCartesianDistance = 0.05;

class IKTrajectoryExecutor {

private:
    ros::NodeHandle node;
    ros::ServiceClient r_ik_client;
    ros::ServiceClient l_ik_client;
    ros::ServiceClient r_fk_client;
    ros::ServiceClient l_fk_client;
//    ros::ServiceClient r_fk_info_client;
//    ros::ServiceClient l_fk_info_client;
    ros::ServiceServer r_service;
    ros::ServiceServer l_service;
    ros::ServiceServer b_service;
    ros::Subscriber joint_state_sub_;
    pr2_controllers_msgs::JointTrajectoryGoal r_goal;
    pr2_controllers_msgs::JointTrajectoryGoal l_goal;
    kinematics_msgs::GetPositionIK::Request ik_request;
    kinematics_msgs::GetPositionIK::Response ik_response;
    kinematics_msgs::GetPositionFK::Request fk_request;
    kinematics_msgs::GetPositionFK::Response fk_response;
//    kinematics_msgs::GetKinematicSolverInfo::Request info_request;
//    kinematics_msgs::GetKinematicSolverInfo::Response info_response;
    //! A model of the robot to see which joints wrap around
    urdf::Model robot_model_;
    //! Flag that tells us if the robot model was initialized successfully
    bool robot_model_initialized_;

    std::map<std::string, double> joint_state_position_map_;
    std::map<std::string, double> joint_state_velocity_map_;

    TrajClient *r_action_client;
    TrajClient *l_action_client;

public:
    IKTrajectoryExecutor() {

        //wait for the various services to be ready
        ROS_INFO("Waiting for services to be ready");
        ros::service::waitForService(RIGHT_ARM_IK_NAME);
        ros::service::waitForService(LEFT_ARM_IK_NAME);
//        ros::service::waitForService(RIGHT_ARM_FK_INFO);
//        ros::service::waitForService(LEFT_ARM_FK_INFO);
        ros::service::waitForService(RIGHT_ARM_FK_NAME);
        ros::service::waitForService(LEFT_ARM_FK_NAME);
        ROS_INFO("Services ready");

        joint_state_sub_ = node.subscribe("joint_states", 1,
                                          &IKTrajectoryExecutor::jointStateCallback, this);

        //create a client function for the IK services
        r_ik_client = node.serviceClient<kinematics_msgs::GetPositionIK> (RIGHT_ARM_IK_NAME, true);
        l_ik_client = node.serviceClient<kinematics_msgs::GetPositionIK> (LEFT_ARM_IK_NAME, true);

        // create a client for the FK info services
//        r_fk_info_client = node.serviceClient<kinematics_msgs::GetKinematicSolverInfo> (RIGHT_ARM_FK_INFO, true);
//        l_fk_info_client = node.serviceClient<kinematics_msgs::GetKinematicSolverInfo> (LEFT_ARM_FK_INFO, true);

        // create a client for the FK services
        r_fk_client = node.serviceClient<kinematics_msgs::GetPositionFK> (RIGHT_ARM_FK_NAME, true);
        l_fk_client = node.serviceClient<kinematics_msgs::GetPositionFK> (LEFT_ARM_FK_NAME, true);

        //tell the joint trajectory action client that we want
        //to spin a thread by default
        r_action_client = new TrajClient("r_arm_controller/joint_trajectory_action", true);
        l_action_client = new TrajClient("l_arm_controller/joint_trajectory_action", true);

        //wait for the action server to come up
        while (ros::ok() && !r_action_client->waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the right arm joint_trajectory_action action server to come up");
        }
        while (ros::ok() && !l_action_client->waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the left arm joint_trajectory_action action server to come up");
        }

        //register services to input desired Cartesian trajectories
        r_service = node.advertiseService("execute_right_arm_cartesian_ik_trajectory",
                &IKTrajectoryExecutor::execute_right_arm_cartesian_ik_trajectory, this);
        l_service = node.advertiseService("execute_left_arm_cartesian_ik_trajectory",
                &IKTrajectoryExecutor::execute_left_arm_cartesian_ik_trajectory, this);
        b_service = node.advertiseService("execute_both_arms_cartesian_ik_trajectory",
                &IKTrajectoryExecutor::execute_both_arms_cartesian_ik_trajectory, this);

        //have to specify the order of the joints we're sending in our
        //joint trajectory goal, even if they're already on the param server
        r_goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
        r_goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
        r_goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
        r_goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
        r_goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
        r_goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
        r_goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

        l_goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
        l_goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
        l_goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
        l_goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
        l_goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
        l_goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
        l_goal.trajectory.joint_names.push_back("l_wrist_roll_joint");

        // for debugging: queue joint names from forward kinematics
//        kinematics_msgs::GetKinematicSolverInfo::Request request;
//        kinematics_msgs::GetKinematicSolverInfo::Response response;
//        if(r_fk_info_client.call(request,response))
//        {
//          for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
//          {
//            ROS_INFO("Joint: %d %s", i,
//             response.kinematic_solver_info.joint_names[i].c_str());
//          }
//        }
//        else
//        {
//          ROS_ERROR("Could not call query service");
//          ros::shutdown();
//          exit(1);
//        }
//        if(l_fk_info_client.call(request,response))
//        {
//          for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
//          {
//            ROS_INFO("Joint: %d %s", i,
//            response.kinematic_solver_info.joint_names[i].c_str());
//          }
//        }
//        else
//        {
//          ROS_ERROR("Could not call query service");
//          ros::shutdown();
//          exit(1);
//        }
        // end debug

        //We need this to unnormalize trajectories
        std::string urdf_xml,full_urdf_xml;
        node.param("urdf_xml", urdf_xml, std::string("robot_description"));
        if(!node.getParam(urdf_xml,full_urdf_xml))
        {
          ROS_ERROR("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
          robot_model_initialized_ = false;
        }
        else
        {
          robot_model_.initString(full_urdf_xml);
          robot_model_initialized_ = true;
        }


    }

    ~IKTrajectoryExecutor() {
        delete r_action_client;
        delete l_action_client;
    }

    //run inverse kinematics on a PoseStamped (7-dof pose
    //(position + quaternion orientation) + header specifying the
    //frame of the pose)
    //tries to stay close to double start_angles[7]
    //returns the solution angles in double solution[7]
    bool run_right_arm_ik(geometry_msgs::PoseStamped pose, double start_angles[7],
            double solution[7], std::string link_name) {

        kinematics_msgs::GetPositionIK::Request ik_request;
        kinematics_msgs::GetPositionIK::Response ik_response;

        ik_request.timeout = ros::Duration(5.0);
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_shoulder_pan_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_shoulder_lift_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_upper_arm_roll_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_elbow_flex_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_forearm_roll_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_wrist_flex_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_wrist_roll_joint");

        ik_request.ik_request.ik_link_name = link_name;

        ik_request.ik_request.pose_stamped = pose;
        ik_request.ik_request.ik_seed_state.joint_state.position.resize(7);

        for (int i = 0; i < 7; i++)
        	// Try to keep up the elbow
        	if (i == 2)
        		ik_request.ik_request.ik_seed_state.joint_state.position[i] = -1.9;
        	else
        		ik_request.ik_request.ik_seed_state.joint_state.position[i] = start_angles[i];

        ROS_INFO("request pose: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f",
                 pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

        bool ik_service_call = r_ik_client.call(ik_request, ik_response);
        if (!ik_service_call) {
            ROS_ERROR("IK service call failed!");
            return 0;
        }
        if (ik_response.error_code.val == ik_response.error_code.SUCCESS) {
            for (int i = 0; i < 7; i++) {
                solution[i] = ik_response.solution.joint_state.position[i];
            }

            ROS_INFO("solution angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f",
                     solution[0], solution[1], solution[2], solution[3], solution[4], solution[5], solution[6]);

            ROS_INFO("IK service call succeeded");
            return 1;
        }
        ROS_INFO("IK service call error code: %d", ik_response.error_code.val);
        return 0;
    }

    bool run_left_arm_ik(geometry_msgs::PoseStamped pose, double start_angles[7],
            double solution[7], std::string link_name) {

        kinematics_msgs::GetPositionIK::Request ik_request;
        kinematics_msgs::GetPositionIK::Response ik_response;

        ik_request.timeout = ros::Duration(5.0);
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_shoulder_pan_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_shoulder_lift_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_upper_arm_roll_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_elbow_flex_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_forearm_roll_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_wrist_flex_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_wrist_roll_joint");

        ik_request.ik_request.ik_link_name = link_name;

        ik_request.ik_request.pose_stamped = pose;
        ik_request.ik_request.ik_seed_state.joint_state.position.resize(7);

        for (int i = 0; i < 7; i++)
        	if (i == 2) // Try to keep up the elbow
        		ik_request.ik_request.ik_seed_state.joint_state.position[i] = 1.9;
        	else
        		ik_request.ik_request.ik_seed_state.joint_state.position[i] = start_angles[i];

        ROS_INFO("request pose: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f",
                 pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

        bool ik_service_call = l_ik_client.call(ik_request, ik_response);
        if (!ik_service_call) {
            ROS_ERROR("IK service call failed!");
            return 0;
        }
        if (ik_response.error_code.val == ik_response.error_code.SUCCESS) {
            for (int i = 0; i < 7; i++) {
                solution[i] = ik_response.solution.joint_state.position[i];
            }

            ROS_INFO("solution angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f",
                    solution[0], solution[1], solution[2], solution[3], solution[4], solution[5], solution[6]);

            ROS_INFO("IK service call succeeded");

            return 1;
        }
        ROS_INFO("IK service call error code: %d", ik_response.error_code.val);
        return 0;
    }

    //figure out where the right arm is now
    void get_current_right_arm_joint_angles(double current_angles[7]) {
        int i;

        //get a single message from the topic 'r_arm_controller/state'
        pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state_msg =
                ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>(
                        "r_arm_controller/state");

        //extract the joint angles from it
        for (i = 0; i < 7; i++) {
            current_angles[i] = state_msg->actual.positions[i];
        }
    }

    //figure out where the right arm is now
    void get_current_left_arm_joint_angles(double current_angles[7]) {
        int i;

        //get a single message from the topic 'l_arm_controller/state'
        pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state_msg =
                ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>(
                        "l_arm_controller/state");

        //extract the joint angles from it
        for (i = 0; i < 7; i++) {
            current_angles[i] = state_msg->actual.positions[i];
        }
    }

    //send a desired joint trajectory to the right arm joint trajectory action
    //and wait for it to finish
    bool execute_right_arm_joint_trajectory(std::vector<double *> joint_trajectory) {
        int i, j;
        int trajectorylength = joint_trajectory.size();

        //get the current joint angles
        double current_angles[7];
        get_current_right_arm_joint_angles(current_angles);

        //fill the goal message with the desired joint trajectory
        r_goal.trajectory.points.resize(trajectorylength + 1);

        //set the first trajectory point to the current position
        r_goal.trajectory.points[0].positions.resize(7);
        r_goal.trajectory.points[0].velocities.resize(7);
        for (j = 0; j < 7; j++) {
            r_goal.trajectory.points[0].positions[j] = current_angles[j];
            r_goal.trajectory.points[0].velocities[j] = 0.0;
        }

        //make the first trajectory point start 0.25 seconds from when we run
        r_goal.trajectory.points[0].time_from_start = ros::Duration(0.25);

        //fill in the rest of the trajectory
        double time_from_start = 0.25;
        for (i = 0; i < trajectorylength; i++) {
            r_goal.trajectory.points[i + 1].positions.resize(7);
            r_goal.trajectory.points[i + 1].velocities.resize(7);

            //fill in the joint positions (velocities of 0 mean that the arm
            //will try to stop briefly at each waypoint)
            for (j = 0; j < 7; j++) {
                r_goal.trajectory.points[i + 1].positions[j] = joint_trajectory[i][j];
                r_goal.trajectory.points[i + 1].velocities[j] = 0.0;
            }

            //compute a desired time for this trajectory point using a max
            //joint velocity
            double max_joint_move = 0;
            for (j = 0; j < 7; j++) {
                double joint_move = fabs(r_goal.trajectory.points[i + 1].positions[j]
                        - r_goal.trajectory.points[i].positions[j]);
                if (joint_move > max_joint_move) max_joint_move = joint_move;
            }
            double seconds = 3.0; //max_joint_move / MAX_JOINT_VEL;
            ROS_INFO("max_joint_move: %0.3f, seconds: %0.3f", max_joint_move, seconds);
            time_from_start += seconds;
            r_goal.trajectory.points[i + 1].time_from_start = ros::Duration(time_from_start);
        }

        //when to start the trajectory
        r_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.25);

        // finally unnormalize to avoid wrist_roll_joint twists
        unnormalizeTrajectory(r_goal);

        ROS_INFO("Sending right arm goal to right arm joint_trajectory_action");

        r_action_client->sendGoal(r_goal);
        r_action_client->waitForResult();

        //get the current joint angles for debugging
        get_current_right_arm_joint_angles(current_angles);

        ROS_INFO("joint angles after trajectory: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n",
                 current_angles[0], current_angles[1], current_angles[2], current_angles[3], current_angles[4], current_angles[5], current_angles[6]);

        // ROS_INFO("Right Arm Action Goal Status: %s", r_action_client->getState());

        if (r_action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Hooray, the right arm finished the trajectory!");
            return 1;
        } else {
	    ROS_ERROR("The right arm failed to execute the trajectory ... :( ... going further ...");
	    // return 0;
            return 1;
        }
    }

    //send a desired joint trajectory to the right arm joint trajectory action
    //and wait for it to finish
    bool execute_left_arm_joint_trajectory(std::vector<double *> joint_trajectory) {
        int i, j;
        int trajectorylength = joint_trajectory.size();

        //get the current joint angles
        double current_angles[7];
        get_current_left_arm_joint_angles(current_angles);

        //fill the goal message with the desired joint trajectory
        l_goal.trajectory.points.resize(trajectorylength + 1);

        //set the first trajectory point to the current position
        l_goal.trajectory.points[0].positions.resize(7);
        l_goal.trajectory.points[0].velocities.resize(7);
        for (j = 0; j < 7; j++) {
            l_goal.trajectory.points[0].positions[j] = current_angles[j];
            l_goal.trajectory.points[0].velocities[j] = 0.0;
        }

        //make the first trajectory point start 0.25 seconds from when we run
        l_goal.trajectory.points[0].time_from_start = ros::Duration(0.25);

        //fill in the rest of the trajectory
        double time_from_start = 0.25;
        for (i = 0; i < trajectorylength; i++) {
            l_goal.trajectory.points[i + 1].positions.resize(7);
            l_goal.trajectory.points[i + 1].velocities.resize(7);

            //fill in the joint positions (velocities of 0 mean that the arm
            //will try to stop briefly at each waypoint)
            for (j = 0; j < 7; j++) {
                l_goal.trajectory.points[i + 1].positions[j] = joint_trajectory[i][j];
                l_goal.trajectory.points[i + 1].velocities[j] = 0.0;
            }

            double max_joint_move;
            double joint_move;
            double seconds;
  //          if ( 0 < i ){
				//compute a desired time for this trajectory point using a max
				//joint velocity
				max_joint_move = 0;
				for (j = 0; j < 7; j++) {

					joint_move =
							fabs(l_goal.trajectory.points[i + 1].positions[j] -
								 l_goal.trajectory.points[i    ].positions[j]);

					if (joint_move > max_joint_move)
						max_joint_move = joint_move;
				}
				seconds = 3.0; // max_joint_move / MAX_JOINT_VEL;
				time_from_start += seconds;
				l_goal.trajectory.points[i + 1].time_from_start = ros::Duration(time_from_start);

	            ROS_INFO("point_id: %d time_from_start: %f max_joint_move: %0.3f, seconds: %0.3f",
	            		 i+1, l_goal.trajectory.points[i + 1].time_from_start.toSec(), max_joint_move, seconds);

//            } else {
//            	l_goal.trajectory.points[i + 1].time_from_start = ros::Duration(3.0);
//                ROS_INFO("point_id: %d time_from_start: %f max_joint_move: %0.3f, seconds: %0.3f",
//                		 i+1, l_goal.trajectory.points[i + 1].time_from_start.toSec(), max_joint_move, seconds);

                l_goal.trajectory.points[i + 1].time_from_start = ros::Duration(time_from_start);
//            }

        }

        //when to start the trajectory
        l_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.25);

        // finally unnormalize to avoid wrist_roll_joint twists
        unnormalizeTrajectory(l_goal);

        ROS_INFO("Sending right left goal to left arm joint_trajectory_action");

        l_action_client->sendGoal(l_goal);
        l_action_client->waitForResult();

        //get the current joint angles for debugging
        get_current_left_arm_joint_angles(current_angles);

        ROS_INFO("joint angles after trajectory: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n",
       		 current_angles[0], current_angles[1], current_angles[2], current_angles[3], current_angles[4], current_angles[5], current_angles[6]);
        
        // ROS_INFO("Left Arm Action Goal Status: %s", l_action_client->getState());

        if (l_action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Hooray, the left arm finished the trajectory!");
            return 1;
        } else {
            ROS_ERROR("The left arm failed to execute the trajectory ... :( ... going further ...");
            // return 0;
            return 1;
        }
    }

    //send a desired joint trajectory to left and right joint trajectory action
    //and wait for it to finish.
    bool execute_both_arms_joint_trajectory( std::vector<double *> rh_joint_trajectory,
                                             std::vector<double *> lh_joint_trajectory,
                                             std::vector<int> stop_at_pose) {

        int rh_trajectory_length = rh_joint_trajectory.size();
        int lh_trajectory_length = lh_joint_trajectory.size();
        int i, j;

        if (rh_trajectory_length != lh_trajectory_length) {
            ROS_ERROR("Different number of trajectory poses for right and left arm");
            return 0;
        }

        //get the current joint angles
        double rh_current_angles[7];
        double lh_current_angles[7];
        get_current_right_arm_joint_angles(rh_current_angles);
        get_current_left_arm_joint_angles(lh_current_angles);

        //fill the goal message with the desired joint trajectory
        r_goal.trajectory.points.resize(rh_trajectory_length + 1);
        l_goal.trajectory.points.resize(lh_trajectory_length + 1);

        //set the first trajectory points to the current position
        r_goal.trajectory.points[0].positions.resize(7);
        r_goal.trajectory.points[0].velocities.resize(7);
        l_goal.trajectory.points[0].positions.resize(7);
        l_goal.trajectory.points[0].velocities.resize(7);

        for (j = 0; j < 7; j++) {
            r_goal.trajectory.points[0].positions[j] = rh_current_angles[j];
            r_goal.trajectory.points[0].velocities[j] = 0.0;
        }
        for (j = 0; j < 7; j++) {
            l_goal.trajectory.points[0].positions[j] = lh_current_angles[j];
            l_goal.trajectory.points[0].velocities[j] = 0.0;
        }

        //make the first trajectory point start 0.25 seconds from when we run
        double time_from_start = 0.25;
        r_goal.trajectory.points[0].time_from_start = ros::Duration(time_from_start);
        l_goal.trajectory.points[0].time_from_start = ros::Duration(time_from_start);

        //fill in the rest of the trajectories
        for (i = 0; i < rh_trajectory_length; i++) {
            r_goal.trajectory.points[i + 1].positions.resize(7);
            r_goal.trajectory.points[i + 1].velocities.resize(7);
            l_goal.trajectory.points[i + 1].positions.resize(7);
            l_goal.trajectory.points[i + 1].velocities.resize(7);

            //fill in the joint positions (velocities of 0 mean that the arm
            //will try to stop briefly at each waypoint)
            for (j = 0; j < 7; j++) {
                r_goal.trajectory.points[i + 1].positions[j] = rh_joint_trajectory[i][j];
                r_goal.trajectory.points[i + 1].velocities[j] = 0.0;
                l_goal.trajectory.points[i + 1].positions[j] = lh_joint_trajectory[i][j];
                l_goal.trajectory.points[i + 1].velocities[j] = 0.0;
            }
        }

       // before we calculate times, unnormalize to avoid wrist_roll_joint twists
        unnormalizeTrajectory( r_goal);
        unnormalizeTrajectory( l_goal);

        // now that all times are set and poses are unnormalized, iterate again and:
        //   1. calculate durations AFTER unnormalizing poses
        //   2. correct joint velocities for poses, the robot isn't supposed to stop.
        //      (leaving out 1st and last pose, because robot MUST stop there)
        for( i = 0; i < rh_trajectory_length; i++) {
            //compute a desired time for this trajectory point using a max
            //joint velocity
            double max_joint_move = 0;
            for (j = 0; j < 7; j++) {
                double rh_joint_move = fabs(r_goal.trajectory.points[i + 1].positions[j]
                                          - r_goal.trajectory.points[i].positions[j]);

                double lh_joint_move = fabs(l_goal.trajectory.points[i + 1].positions[j]
                                          - l_goal.trajectory.points[i].positions[j]);

                if (rh_joint_move > max_joint_move)
                	max_joint_move = rh_joint_move;

                if (lh_joint_move > max_joint_move)
                	max_joint_move = lh_joint_move;
            }

            double seconds = 3.0;//max_joint_move / MAX_JOINT_VEL;

            ROS_INFO("max_joint_move: %0.3f, seconds: %0.3f", max_joint_move, seconds);

            time_from_start += seconds;

            r_goal.trajectory.points[i + 1].time_from_start = ros::Duration(time_from_start);
            l_goal.trajectory.points[i + 1].time_from_start = ros::Duration(time_from_start);
        }

        for( i = 1; i < (rh_trajectory_length -1); i++) {
            if( stop_at_pose[i] == 0) {

                double time_step = r_goal.trajectory.points[i + 2].time_from_start.toSec()
                                 - r_goal.trajectory.points[i].time_from_start.toSec();

                for( j = 0; j < 7; ++j) {
                    r_goal.trajectory.points[i + 1].velocities[j] =
		            ( r_goal.trajectory.points[i + 2].positions[j] -
                      r_goal.trajectory.points[i    ].positions[j]) / time_step;
                    //std::cout << "VELOCITY OF R JOINT NO " << j << " : " << r_goal.trajectory.points[i + 1].velocities[j] << std::endl;
                    l_goal.trajectory.points[i + 1].velocities[j] =
                    ( l_goal.trajectory.points[i + 2].positions[j] -
                      l_goal.trajectory.points[i    ].positions[j]) / time_step;
                    //std::cout << "VELOCITY OF L JOINT NO " << j << " : " << l_goal.trajectory.points[i + 1].velocities[j] << std::endl;
                }

            }
        }

        //when to start the trajectory
        r_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.25);
        l_goal.trajectory.header.stamp = r_goal.trajectory.header.stamp;

        ROS_INFO("Sending goal to both arm's joint_trajectory_action");

        r_action_client->sendGoal(r_goal);
        l_action_client->sendGoal(l_goal);

        r_action_client->waitForResult();
        l_action_client->waitForResult();

        if (r_action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED &&
            l_action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Hooray, both arms finished the trajectories!");
            return 1;
        } else {
            ROS_ERROR("One of the arms failed to reach the goal ... :( ... going further ...");
            // return 0;
            return 1;
        }
    }


    //service function for execute_right_arm_cartesian_ik_trajectory
    bool execute_right_arm_cartesian_ik_trajectory(
            two_hand_ik_trajectory_executor::ExecuteRightArmCartesianIKTrajectory::Request &req,
            two_hand_ik_trajectory_executor::ExecuteRightArmCartesianIKTrajectory::Response &res) {

        int trajectory_length = req.poses.size();
        int i, j;

        //IK takes in Cartesian poses stamped with the frame they belong to
        geometry_msgs::PoseStamped stamped_pose;
        stamped_pose.header = req.header;
        stamped_pose.header.stamp = ros::Time::now();
        bool success;
        std::vector<double *> joint_trajectory;

        //get the current joint angles (to find ik solutions close to)
        double last_angles[7];
        get_current_right_arm_joint_angles(last_angles);

        //find IK solutions for each point along the trajectory
        //and stick them into joint_trajectory
        for (i = 0; i < trajectory_length; i++) {

            stamped_pose.pose = req.poses[i];
            double *trajectory_point = new double[7];
            success = run_right_arm_ik(stamped_pose, last_angles, trajectory_point,
                    "r_wrist_roll_link");
            joint_trajectory.push_back(trajectory_point);

            if (!success) {
                ROS_ERROR("IK solution not found for trajectory point number %d!\n", i);
                return 0;
            }
            for (j = 0; j < 7; j++)
                last_angles[j] = trajectory_point[j];
        }

        //run the resulting joint trajectory
        ROS_INFO("executing right arm joint trajectory");
        success = execute_right_arm_joint_trajectory(joint_trajectory);
        res.success = success;

        return success;
    }

    //service function for execute_left_arm_cartesian_ik_trajectory
    bool execute_left_arm_cartesian_ik_trajectory(
            two_hand_ik_trajectory_executor::ExecuteLeftArmCartesianIKTrajectory::Request &req,
            two_hand_ik_trajectory_executor::ExecuteLeftArmCartesianIKTrajectory::Response &res) {

        int trajectory_length = req.poses.size();
        int i, j;

        //IK takes in Cartesian poses stamped with the frame they belong to
        geometry_msgs::PoseStamped stamped_pose;
        stamped_pose.header = req.header;
        stamped_pose.header.stamp = ros::Time::now();
        bool success;
        std::vector<double *> joint_trajectory;

        //get the current joint angles (to find ik solutions close to)
        double last_angles[7];
        get_current_left_arm_joint_angles(last_angles);

        //find IK solutions for each point along the trajectory
        //and stick them into joint_trajectory
        for (i = 0; i < trajectory_length; i++) {

            stamped_pose.pose = req.poses[i];
            double *trajectory_point = new double[7];
            success = run_left_arm_ik(stamped_pose, last_angles, trajectory_point,
                    "l_wrist_roll_link");
            joint_trajectory.push_back(trajectory_point);

            if (!success) {
                ROS_ERROR("IK solution not found for trajectory point number %d!\n", i);
                return 0;
            }
            for (j = 0; j < 7; j++)
                last_angles[j] = trajectory_point[j];
        }

        //run the resulting joint trajectory
        ROS_INFO("executing left arm joint trajectory");
        success = execute_left_arm_joint_trajectory(joint_trajectory);
        res.success = success;

        return success;
    }


    // helper functions to get Cartesian pose from joint angles with forward kinematic
    int get_current_right_arm_cartesian_pose(double angles[7], geometry_msgs::Pose& pose) {

        // create the service messages
        kinematics_msgs::GetPositionFK::Request fk_request;
        kinematics_msgs::GetPositionFK::Response fk_response;

        fk_request.header.frame_id = "base_link";

        fk_request.fk_link_names.resize(1);
        fk_request.fk_link_names[0] = "r_wrist_roll_link";

        fk_request.robot_state.joint_state.name.resize(7);
        fk_request.robot_state.joint_state.name[0] = "r_shoulder_pan_joint";
        fk_request.robot_state.joint_state.name[1] = "r_shoulder_lift_joint";
        fk_request.robot_state.joint_state.name[2] = "r_upper_arm_roll_joint";
        fk_request.robot_state.joint_state.name[3] = "r_elbow_flex_joint";
        fk_request.robot_state.joint_state.name[4] = "r_forearm_roll_joint";
        fk_request.robot_state.joint_state.name[5] = "r_wrist_flex_joint";
        fk_request.robot_state.joint_state.name[6] = "r_wrist_roll_joint";

        fk_request.robot_state.joint_state.position.resize(7);
        for (int i = 0; i < 7; i++)
            fk_request.robot_state.joint_state.position[i] = angles[i];

        // call the service
        bool fk_service_call = r_fk_client.call(fk_request, fk_response);

        if (!fk_service_call) {
            ROS_ERROR("FK service call for right hand failed!");
            return 0;
        }

        if (fk_response.error_code.val == fk_response.error_code.SUCCESS) {
            pose = fk_response.pose_stamped[0].pose;
            ROS_INFO("FK service call succeeded");
            return 1;
        }
        else {
            ROS_INFO("FK service call error code: %d", fk_response.error_code.val);
            return 0;
        }
    }


    int get_current_left_arm_cartesian_pose(double angles[7], geometry_msgs::Pose& pose) {

         // create the service messages
         kinematics_msgs::GetPositionFK::Request fk_request;
         kinematics_msgs::GetPositionFK::Response fk_response;

         fk_request.header.frame_id = "base_link";

         fk_request.fk_link_names.resize(1);
         fk_request.fk_link_names[0] = "l_wrist_roll_link";

         fk_request.robot_state.joint_state.name.resize(7);
         fk_request.robot_state.joint_state.name[0] = "l_shoulder_pan_joint";
         fk_request.robot_state.joint_state.name[1] = "l_shoulder_lift_joint";
         fk_request.robot_state.joint_state.name[2] = "l_upper_arm_roll_joint";
         fk_request.robot_state.joint_state.name[3] = "l_elbow_flex_joint";
         fk_request.robot_state.joint_state.name[4] = "l_forearm_roll_joint";
         fk_request.robot_state.joint_state.name[5] = "l_wrist_flex_joint";
         fk_request.robot_state.joint_state.name[6] = "l_wrist_roll_joint";

         fk_request.robot_state.joint_state.position.resize(7);
         for (int i = 0; i < 7; i++)
             fk_request.robot_state.joint_state.position[i] = angles[i];

         // call the service
         bool fk_service_call = l_fk_client.call(fk_request, fk_response);

         if (!fk_service_call) {
             ROS_ERROR("FK service call for left hand failed!");
             return 0;
         }

         if (fk_response.error_code.val == fk_response.error_code.SUCCESS) {
             pose = fk_response.pose_stamped[0].pose;
             ROS_INFO("FK service call succeeded");
             return 1;
         }
         else {
             ROS_INFO("FK service call error code: %d", fk_response.error_code.val);
             return 0;
         }
    }


    // stole this function from pr2_teleop_general
    void unnormalizeTrajectory(pr2_controllers_msgs::JointTrajectoryGoal& traj) const {

      std::vector<double> current_values;
      std::vector<bool> wraparound;
      trajectory_msgs::JointTrajectory input_trajectory = traj.trajectory;

      for (size_t i=0; i<input_trajectory.joint_names.size(); i++)
      {
        std::string name = input_trajectory.joint_names[i];

        double pos;

        if(!getJointPosition(name, pos)) {
          ROS_WARN_STREAM("Can't unnormalize as no current joint state for " << name);
          return;
        }

        //first waypoint is unnormalized relative to current joint states
        current_values.push_back(pos);

        boost::shared_ptr<const urdf::Joint> joint = robot_model_.getJoint(name);
        if (joint.get() == NULL)
        {
          ROS_ERROR("Joint name %s not found in urdf model", name.c_str());
          return;
        }
        if (joint->type == urdf::Joint::CONTINUOUS) {
          wraparound.push_back(true);
        }
        else {
          wraparound.push_back(false);
        }
      }

      trajectory_msgs::JointTrajectory unnormalized_trajectory = input_trajectory;
      for (size_t i=0; i<unnormalized_trajectory.points.size(); i++)
      {
        for (size_t j=0; j<unnormalized_trajectory.points[i].positions.size(); j++)
        {
          if(!wraparound[j]){
            continue;
          }
          double current = current_values[j];
          double traj = unnormalized_trajectory.points[i].positions[j];
          while ( current - traj > M_PI ) traj += 2*M_PI;
          while ( traj - current > M_PI ) traj -= 2*M_PI;
          ROS_DEBUG("Normalizing joint %s from %f to %f", unnormalized_trajectory.joint_names.at(j).c_str(),
                    unnormalized_trajectory.points[i].positions[j], traj);
          unnormalized_trajectory.points[i].positions[j] = traj;
          //all other waypoints are unnormalized relative to the previous waypoint
          current_values[j] = traj;
        }
      }
      traj.trajectory = unnormalized_trajectory;
    }

    //helper function for unnormalizeTrajectory, also stolen from pr2_teleop_general
    bool getJointPosition(const std::string& name, double& pos) const {
      if(joint_state_position_map_.find(name) == joint_state_position_map_.end()) {
        return false;
      }
      pos = joint_state_position_map_.find(name)->second;
      return true;
    }

    // same as above
    bool getJointVelocity(const std::string& name, double& vel) const {
      if(joint_state_velocity_map_.find(name) == joint_state_velocity_map_.end()) {
        return false;
      }
      vel = joint_state_velocity_map_.find(name)->second;
      return true;
    }


    //callback funktion for JointStatePublisher, also stolen from pr2_teleop_general
    void jointStateCallback(const sensor_msgs::JointStateConstPtr &jointState)
    {
      for(unsigned int i = 0; i < jointState->name.size(); i++) {
        joint_state_position_map_[jointState->name[i]] = jointState->position[i];
        joint_state_velocity_map_[jointState->name[i]] = jointState->velocity[i];
        //if(jointState->name[i] == "r_wrist_roll_joint") {
        //  ROS_INFO_STREAM("Right wrist roll pos " <<  jointState->position[i] << " vel " <<  jointState->velocity[i]);
        //}
      }
    }


    //service function for execute_both_arms_cartesian_ik_trajectory
    // we apply the WORKAROUND for our "spin-hand-problem" in this function
    bool execute_both_arms_cartesian_ik_trajectory(
            two_hand_ik_trajectory_executor::ExecuteBothArmsCartesianIKTrajectory::Request &req,
            two_hand_ik_trajectory_executor::ExecuteBothArmsCartesianIKTrajectory::Response &res) {

        int rh_trajectory_length = req.rh_poses.size();
        int lh_trajectory_length = req.lh_poses.size();
        int i, j;

        if (rh_trajectory_length != lh_trajectory_length) {
            ROS_ERROR("Different number of trajectory poses for right and left arm");
            return 0;
        }

        // the array, where we put our trajectory points
        std::vector<double*> rh_joint_trajectory;
        std::vector<double*> lh_joint_trajectory;
        std::vector<int> stop_at_pose;

        // IK takes in Cartesian poses stamped with the frame they belong to
        geometry_msgs::PoseStamped rh_stamped_pose;
        geometry_msgs::PoseStamped lh_stamped_pose;

        rh_stamped_pose.header = req.header;
        lh_stamped_pose.header = req.header;
        rh_stamped_pose.header.stamp = ros::Time::now();
        lh_stamped_pose.header.stamp = rh_stamped_pose.header.stamp; //Use same timestamp for sync

        // the variables where we save function returns
        bool rh_ik_success;
        bool lh_ik_success;
        bool exec_traj_success;

        //get the current joint angles (to find ik solutions close to)
        double rh_last_angles[7];
        double lh_last_angles[7];
        get_current_right_arm_joint_angles(rh_last_angles);
        get_current_left_arm_joint_angles(lh_last_angles);

        //find IK solutions for each point along the trajectory
        //and put them into joint_trajectory
        for (i = 0; i < rh_trajectory_length; i++) {

            rh_stamped_pose.pose = req.rh_poses[i];
            lh_stamped_pose.pose = req.lh_poses[i];

            double* rh_trajectory_point = new double[7];
            double* lh_trajectory_point = new double[7];

            rh_ik_success = run_right_arm_ik(rh_stamped_pose, rh_last_angles,
                    rh_trajectory_point, "r_wrist_roll_link");
            lh_ik_success = run_left_arm_ik(lh_stamped_pose, lh_last_angles,
                    lh_trajectory_point, "l_wrist_roll_link");

            rh_joint_trajectory.push_back(rh_trajectory_point);
            lh_joint_trajectory.push_back(lh_trajectory_point);
            stop_at_pose.push_back( req.stop_at_pose[i]);

            if (!rh_ik_success) {
                ROS_ERROR("IK solution not found for right hand trajectory point number %d!\n", i);
                return 0;
            }
            else if (!lh_ik_success) {
                ROS_ERROR("IK solution not found for left hand trajectory point number %d!\n", i);
                return 0;
            }

            for (j = 0; j < 7; j++)
                rh_last_angles[j] = rh_trajectory_point[j];
            for (j = 0; j < 7; j++)
                lh_last_angles[j] = lh_trajectory_point[j];

        }

        //run the resulting joint trajectory
        ROS_INFO("executing both arms joint trajectory");
        exec_traj_success = execute_both_arms_joint_trajectory(rh_joint_trajectory,
                                                               lh_joint_trajectory,
                                                               stop_at_pose);
        res.success = exec_traj_success;

        return exec_traj_success;
    }

};


    int main(int argc, char** argv) {

        //init ROS node
        ros::init(argc, argv, "cartesian_ik_trajectory_executor");

        IKTrajectoryExecutor ik_traj_exec = IKTrajectoryExecutor();

        ROS_INFO("Waiting for cartesian trajectories to execute");
        ros::spin();

        return 0;
    }
