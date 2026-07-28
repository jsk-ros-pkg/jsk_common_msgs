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


#ifndef POSEDETECTION_MSGS_FEATURE0D_TO_IMAGE_H_
#define POSEDETECTION_MSGS_FEATURE0D_TO_IMAGE_H_

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
#include <sensor_msgs/Image.h>
#include <posedetection_msgs/ImageFeature0D.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <posedetection_msgs/msg/image_feature0_d.hpp>

#include <memory>
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#if ROS_VERSION_MAJOR == 1
#include <boost/shared_ptr.hpp>
#endif

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#if ROS_VERSION_MAJOR == 1
#if BOOST_VERSION < 106000  // since 1.60.0, boost uses placeholders namesapce for _1,_2...
#ifndef BOOST_PLAEHOLDERS
#define BOOST_PLAEHOLDERS
namespace boost
{
namespace placeholders
{
extern boost::arg<1> _1;
extern boost::arg<2> _2;
extern boost::arg<3> _3;
extern boost::arg<4> _4;
extern boost::arg<5> _5;
extern boost::arg<6> _6;
extern boost::arg<7> _7;
extern boost::arg<8> _8;
extern boost::arg<9> _9;
}  // namespace placeholders
}  // namespace boost
#endif  // BOOST_PLAEHOLDERS
#endif  // BOOST_VERSION < 106000
#endif

namespace posedetection_msgs
{
  cv::Mat draw_features(const cv::Mat src,
                        const std::vector<float> positions,
                        const std::vector<float> scales,
                        const std::vector<float> orientations)
  {
    cv::Mat dst;
    src.copyTo(dst);
    for(size_t i = 0; i < positions.size()/2; ++i) {
      float scale = i < scales.size() ? scales[i] : 10.0;
      cv::Point center = cv::Point(positions[2*i+0], positions[2*i+1]);
      cv::circle(dst, center, scale, CV_RGB(0,255,0));
      if( i < orientations.size() ) {
        // draw line indicating orientation
        cv::Point end_pt = cv::Point(center.x+std::cos(orientations[i])*scale,
            center.y+std::sin(orientations[i])*scale);
        cv::line(dst, center, end_pt, CV_RGB(255,0,0));
      }
    }
    return dst;
  }

  class Feature0DToImage
  {
  public:
#if ROS_VERSION_MAJOR == 1
    ros::NodeHandle _node;
    ros::Publisher _pub;
    ros::Subscriber _sub_imagefeature;
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image,
    posedetection_msgs::Feature0D
    > SyncPolicy;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > _sync;
    message_filters::Subscriber<sensor_msgs::Image> _sub_image;
    message_filters::Subscriber<posedetection_msgs::Feature0D> _sub_feature;

    Feature0DToImage();
#else
    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub;
    rclcpp::Subscription<posedetection_msgs::msg::ImageFeature0D>::SharedPtr _sub_imagefeature;
    typedef message_filters::sync_policies::ExactTime<
        sensor_msgs::msg::Image,
        posedetection_msgs::msg::Feature0D
    > SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> _sync;
    message_filters::Subscriber<sensor_msgs::msg::Image> _sub_image;
    message_filters::Subscriber<posedetection_msgs::msg::Feature0D> _sub_feature;

    explicit Feature0DToImage(rclcpp::Node::SharedPtr node);
#endif
    virtual ~Feature0DToImage();
#if ROS_VERSION_MAJOR == 1
    void imagefeature_cb(const posedetection_msgs::ImageFeature0DConstPtr& msg_ptr);
    void imagefeature_cb(const sensor_msgs::ImageConstPtr& image_msg,
                         const posedetection_msgs::Feature0DConstPtr& feature_msg);
#else
    void imagefeature_cb(const posedetection_msgs::msg::ImageFeature0D::ConstSharedPtr msg_ptr);
    void imagefeature_cb(
      const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
      const posedetection_msgs::msg::Feature0D::ConstSharedPtr & feature_msg);
#endif
  };
}

#endif
