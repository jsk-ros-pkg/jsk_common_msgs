// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#ifndef POSEDETECTION_MSGS_FEATURE0D_VIEW_H_
#define POSEDETECTION_MSGS_FEATURE0D_VIEW_H_

// roscpp is only on the include path in a ROS1 (catkin) build; ROS2
// (ament_cmake) builds this package against rclcpp instead. If it's
// there, pull it in and let its own ROS_VERSION_MAJOR (from
// ros/common.h) tell ROS1 and ROS2 apart; if it's not there,
// ROS_VERSION_MAJOR stays undefined and reads as 0 below. __has_include
// itself isn't recognized by old compilers (e.g. GCC 4.8 on indigo), so
// fall back to assuming ROS1 there rather than letting the #if fail to
// parse -- only ROS1 distros predating __has_include support exist.
#if defined(__has_include)
#if __has_include(<ros/ros.h>)
#include <ros/ros.h>
#endif
#else
#include <ros/ros.h>
#endif

#if ROS_VERSION_MAJOR == 1
#include <posedetection_msgs/ImageFeature0D.h>
#include <cv_bridge/cv_bridge.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <posedetection_msgs/msg/image_feature0_d.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <string>
#endif

namespace posedetection_msgs
{
  class Feature0DView
  {
  public:
#if ROS_VERSION_MAJOR == 1
    ros::NodeHandle _node;
    ros::Subscriber _sub;
#else
    rclcpp::Node::SharedPtr _node;
    rclcpp::Subscription<posedetection_msgs::msg::ImageFeature0D>::SharedPtr _sub;
#endif
    std::string _window_name;
    cv_bridge::CvImage _bridge;

#if ROS_VERSION_MAJOR == 1
    Feature0DView();
#else
    explicit Feature0DView(rclcpp::Node::SharedPtr node);
#endif
    virtual ~Feature0DView();
#if ROS_VERSION_MAJOR == 1
    void image_cb(const posedetection_msgs::ImageFeature0DConstPtr& msg_ptr);
#else
    void image_cb(const posedetection_msgs::msg::ImageFeature0D::ConstSharedPtr msg_ptr);
#endif
  };
}

#endif
