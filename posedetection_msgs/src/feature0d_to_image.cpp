// vim: set tabstop=4 shiftwidth=4:
// Copyright (C) 2008-2009 Rosen Diankov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "posedetection_msgs/feature0d_to_image.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <posedetection_msgs/msg/image_feature0_d.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/synchronizer.h>

namespace posedetection_msgs
{
Feature0DToImage::Feature0DToImage(const rclcpp::NodeOptions & options)
: Node("feature0d_to_image", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  _pub = create_publisher<sensor_msgs::msg::Image>("~/output", 1);
  _sub_image.subscribe(this, "~/image", rclcpp::QoS(1).get_rmw_qos_profile());
  _sub_feature.subscribe(this, "~/Feature0D", rclcpp::QoS(1).get_rmw_qos_profile());
  _sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(100);
  _sync->connectInput(_sub_image, _sub_feature);
  _sync->registerCallback(std::bind(&Feature0DToImage::image_with_feature_cb, this, _1, _2));
  _sub_imagefeature = create_subscription<posedetection_msgs::msg::ImageFeature0D>(
    "~/ImageFeature0D", 1, std::bind(&Feature0DToImage::imagefeature_cb, this, _1));
}

void Feature0DToImage::image_with_feature_cb(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const posedetection_msgs::msg::Feature0D::ConstSharedPtr & feature_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
    cv::Mat image = draw_features(
      cv_ptr->image,
      feature_msg->positions,
      feature_msg->scales,
      feature_msg->orientations);
    _pub->publish(*cv_bridge::CvImage(cv_ptr->header, "bgr8", image).toImageMsg());
  } catch (cv_bridge::Exception & error) {
    RCLCPP_WARN(get_logger(), "bad frame");
    return;
  }
}

void Feature0DToImage::imagefeature_cb(
  const posedetection_msgs::msg::ImageFeature0D::ConstSharedPtr msg_ptr)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg_ptr->image, "bgr8");
    cv::Mat image = draw_features(
      cv_ptr->image,
      msg_ptr->features.positions,
      msg_ptr->features.scales,
      msg_ptr->features.orientations);
    _pub->publish(*cv_bridge::CvImage(cv_ptr->header, "bgr8", image).toImageMsg());
  } catch (cv_bridge::Exception & error) {
    RCLCPP_WARN(get_logger(), "bad frame");
    return;
  }
}
}  // namespace posedetection_msgs

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(posedetection_msgs::Feature0DToImage)
