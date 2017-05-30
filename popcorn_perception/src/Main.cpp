/*
 * Copyright (c) 2017, Mihaela Popescu <popescu@uni-bremen.de>
 * All rights reserved.
 *
 */
 
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <popcorn_perception/CPerceiveActionServer.h>

using namespace std;



int main(int argc, char** argv)
{
  ros::init(argc, argv, "perceive_action");

  CPerceiveActionServer perceive("perceive_action");
  ros::spin();

  return 0;
}