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
#include "posedetection_msgs/feature0d_to_image.h"

#if ROS_VERSION_MAJOR == 1
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <posedetection_msgs/ImageFeature0D.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <posedetection_msgs/msg/image_feature0_d.hpp>
#endif

#include <opencv2/highgui/highgui.hpp>
#if ROS_VERSION_MAJOR == 1
#include <boost/shared_ptr.hpp>
#endif

#if ROS_VERSION_MAJOR == 1
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <message_filters/synchronizer.h>

namespace posedetection_msgs
{
#if ROS_VERSION_MAJOR == 1
    Feature0DToImage::Feature0DToImage()
    {
        ros::NodeHandle local_nh("~");

        _pub = _node.advertise<sensor_msgs::Image>(local_nh.resolveName("output"), 1);
        _sub_image.subscribe(_node, "image", 1);
        _sub_feature.subscribe(_node, "Feature0D", 1);
        _sync = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
        _sync->connectInput(_sub_image, _sub_feature);
        _sync->registerCallback(boost::bind(&Feature0DToImage::imagefeature_cb, this,
            boost::placeholders::_1, boost::placeholders::_2));
        _sub_imagefeature = _node.subscribe("ImageFeature0D", 1, &Feature0DToImage::imagefeature_cb, this);
    }
#else
    Feature0DToImage::Feature0DToImage(rclcpp::Node::SharedPtr node)
    : _node(node)
    {
      using std::placeholders::_1;
      using std::placeholders::_2;
      _pub = _node->create_publisher<sensor_msgs::msg::Image>("~/output", 1);
      _sub_image.subscribe(_node.get(), "image", rclcpp::QoS(1).get_rmw_qos_profile());
      _sub_feature.subscribe(_node.get(), "Feature0D", rclcpp::QoS(1).get_rmw_qos_profile());
      _sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(100);
      _sync->connectInput(_sub_image, _sub_feature);
      _sync->registerCallback(std::bind(
          static_cast<void (Feature0DToImage::*)(
            const sensor_msgs::msg::Image::ConstSharedPtr &,
            const posedetection_msgs::msg::Feature0D::ConstSharedPtr &)>(
            &Feature0DToImage::imagefeature_cb), this, _1, _2));
      _sub_imagefeature = _node->create_subscription<posedetection_msgs::msg::ImageFeature0D>(
        "ImageFeature0D", 1, std::bind(
          static_cast<void (Feature0DToImage::*)(
            const posedetection_msgs::msg::ImageFeature0D::ConstSharedPtr)>(
            &Feature0DToImage::imagefeature_cb), this, _1));
    }
#endif
    Feature0DToImage::~Feature0DToImage() {}

    void Feature0DToImage::imagefeature_cb(
#if ROS_VERSION_MAJOR == 1
                                           const sensor_msgs::ImageConstPtr& image_msg,
                                           const posedetection_msgs::Feature0DConstPtr& feature_msg)
#else
                                           const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
                                           const posedetection_msgs::msg::Feature0D::ConstSharedPtr & feature_msg)
#endif
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
            cv::Mat image = draw_features(cv_ptr->image,
                                          feature_msg->positions,
                                          feature_msg->scales,
                                          feature_msg->orientations);
#if ROS_VERSION_MAJOR == 1
            _pub.publish(cv_bridge::CvImage(cv_ptr->header, "bgr8", image));
#else
            _pub->publish(*cv_bridge::CvImage(cv_ptr->header, "bgr8", image).toImageMsg());
#endif
        } catch (cv_bridge::Exception& error) {
#if ROS_VERSION_MAJOR == 1
            ROS_WARN("bad frame");
#else
            RCLCPP_WARN(_node->get_logger(), "bad frame");
#endif
            return;
        }
    }

#if ROS_VERSION_MAJOR == 1
    void Feature0DToImage::imagefeature_cb(const posedetection_msgs::ImageFeature0DConstPtr& msg_ptr)
#else
    void Feature0DToImage::imagefeature_cb(const posedetection_msgs::msg::ImageFeature0D::ConstSharedPtr msg_ptr)
#endif
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg_ptr->image, "bgr8");
            cv::Mat image = draw_features(cv_ptr->image,
                                          msg_ptr->features.positions,
                                          msg_ptr->features.scales,
                                          msg_ptr->features.orientations);
#if ROS_VERSION_MAJOR == 1
            _pub.publish(cv_bridge::CvImage(cv_ptr->header, "bgr8", image));
#else
            _pub->publish(*cv_bridge::CvImage(cv_ptr->header, "bgr8", image).toImageMsg());
#endif
        } catch (cv_bridge::Exception& error) {
#if ROS_VERSION_MAJOR == 1
            ROS_WARN("bad frame");
#else
            RCLCPP_WARN(_node->get_logger(), "bad frame");
#endif
            return;
        }
    }
}

#if ROS_VERSION_MAJOR == 1
int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature0d_to_image");
    boost::shared_ptr<posedetection_msgs::Feature0DToImage> node(new posedetection_msgs::Feature0DToImage());
    ros::spin();
    return 0;
}
#else
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("feature0d_to_image");
  auto converter = std::make_shared<posedetection_msgs::Feature0DToImage>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
