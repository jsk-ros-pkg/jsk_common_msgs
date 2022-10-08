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


#ifndef POSEDETECTION_MSGS__FEATURE0D_TO_IMAGE_HPP_
#define POSEDETECTION_MSGS__FEATURE0D_TO_IMAGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <posedetection_msgs/msg/image_feature0_d.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <memory>
#include <vector>

namespace posedetection_msgs
{
cv::Mat draw_features(
  const cv::Mat src,
  const std::vector<float> positions,
  const std::vector<float> scales,
  const std::vector<float> orientations)
{
  cv::Mat dst;
  src.copyTo(dst);
  for (size_t i = 0; i < positions.size() / 2; ++i) {
    float scale = i < scales.size() ? scales[i] : 10.0;
    cv::Point center = cv::Point(positions[2 * i + 0], positions[2 * i + 1]);
    cv::circle(dst, center, scale, CV_RGB(0, 255, 0));
    if (i < orientations.size() ) {
      // draw line indicating orientation
      cv::Point end_pt = cv::Point(
        center.x + std::cos(orientations[i]) * scale,
        center.y + std::sin(orientations[i]) * scale);
      cv::line(dst, center, end_pt, CV_RGB(255, 0, 0));
    }
  }
  return dst;
}

class Feature0DToImage : public rclcpp::Node
{
public:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub;
  rclcpp::Subscription<posedetection_msgs::msg::ImageFeature0D>::SharedPtr _sub_imagefeature;
  typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::msg::Image,
      posedetection_msgs::msg::Feature0D
  > SyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> _sync;
  message_filters::Subscriber<sensor_msgs::msg::Image> _sub_image;
  message_filters::Subscriber<posedetection_msgs::msg::Feature0D> _sub_feature;

  explicit Feature0DToImage(const rclcpp::NodeOptions & options);
  void imagefeature_cb(const posedetection_msgs::msg::ImageFeature0D::ConstSharedPtr msg_ptr);
  void image_with_feature_cb(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const posedetection_msgs::msg::Feature0D::ConstSharedPtr & feature_msg);
};
}  // namespace posedetection_msgs

#endif  // POSEDETECTION_MSGS__FEATURE0D_TO_IMAGE_HPP_
