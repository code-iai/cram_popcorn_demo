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

//! Geometry helpers

#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include <ros/ros.h>

#include <tf/transform_listener.h>

class Geometry
{
private:

    static tf::TransformListener *listener_;

    static Geometry *instance;

    Geometry();

    ~Geometry();

public:

    static Geometry* getInstance()
    {
        if (!instance)instance = new Geometry();
        return instance;
    }

    static void init();

    static tf::Stamped<tf::Pose> getPoseIn(const char target_frame[], tf::Stamped<tf::Pose> src);

    static tf::Stamped<tf::Pose> getPose(const char target_frame[], const char lookup_frame[], ros::Time tm = ros::Time(0));

    static tf::Stamped<tf::Pose> rotateAroundPose(tf::Stamped<tf::Pose> toolPose, 
                                                  tf::Stamped<tf::Pose> pivot,
                                                  double r_x, double r_y, double r_z);
    
    static tf::Stamped<tf::Pose> rotateAroundPose(tf::Stamped<tf::Pose> toolPose, tf::Stamped<tf::Pose> pivot, tf::Quaternion qa);

    static tf::Stamped<tf::Pose> approach(tf::Stamped<tf::Pose> toolPose, double dist = 0.1);
    static tf::Stamped<tf::Pose> high(tf::Stamped<tf::Pose> toolPose, double dist = 0.1);

    static tf::Stamped<tf::Pose> make_pose(double x, double y, double z,
                                           double ox, double oy, double oz, double ow,
                                           const char target_frame[]);

    static tf::Stamped<tf::Pose> make_pose(const tf::Transform &trans, const char target_frame[]);

};


#endif
