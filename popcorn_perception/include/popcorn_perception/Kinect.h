
/*
 * Copyright (c) 2011, Thomas Ruehr <ruehr@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef __KINECT_H__
#define __KINECT_H__

#include <ros/ros.h>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class Kinect
{
private:

  Kinect()
  {
    rgb_optical_frame_ = "head_mount_kinect_rgb_optical_frame";
    rgb_topic_ = "/kinect_head/depth_registered/points";
  };

  ~Kinect(){};

  static Kinect *instance_;

public:

  void getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                std::string frame_id="/map");

  /*
  void getCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                std::string frame_id="/map",
                ros::Time after = ros::Time(0, 0),
                ros::Time *tm = 0);
  */

  static Kinect* getInstance()
  {
    if (!instance_)
      instance_ = new Kinect();

    return instance_;
  }

  std::string rgb_optical_frame_;
  std::string rgb_topic_;
};

#endif
