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
#include "posedetection_msgs/feature0d_view.h"
#include "posedetection_msgs/feature0d_to_image.h"

#if ROS_VERSION_MAJOR == 1
#include <ros/ros.h>
#include <posedetection_msgs/ImageFeature0D.h>
#else
#include <rclcpp/rclcpp.hpp>
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

namespace posedetection_msgs
{
#if ROS_VERSION_MAJOR == 1
    Feature0DView::Feature0DView()
    {
        std::string topic = _node.resolveName("ImageFeature0D");
        ros::NodeHandle local_nh("~");
        local_nh.param("window_name", _window_name, topic);
        bool autosize;
        local_nh.param("autosize", autosize, false);

        _sub = _node.subscribe("ImageFeature0D",1,&Feature0DView::image_cb,this);
	cv::namedWindow(_window_name.c_str(), autosize ? cv::WINDOW_AUTOSIZE : 0);
	cv::startWindowThread();
    }
#else
    Feature0DView::Feature0DView(rclcpp::Node::SharedPtr node)
    : _node(node)
    {
      using std::placeholders::_1;
      _window_name = _node->declare_parameter("window_name", "ImageFeature0D");
      bool autosize = _node->declare_parameter("autosize", false);

      _sub = _node->create_subscription<posedetection_msgs::msg::ImageFeature0D>(
        "ImageFeature0D", 1, std::bind(&Feature0DView::image_cb, this, _1));
      cv::namedWindow(_window_name.c_str(), autosize ? cv::WINDOW_AUTOSIZE : 0);
      cv::startWindowThread();
    }
#endif
    Feature0DView::~Feature0DView() {}

#if ROS_VERSION_MAJOR == 1
    void Feature0DView::image_cb(const posedetection_msgs::ImageFeature0DConstPtr& msg_ptr)
#else
    void Feature0DView::image_cb(const posedetection_msgs::msg::ImageFeature0D::ConstSharedPtr msg_ptr)
#endif
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg_ptr->image, "bgr8");
            cv::Mat image = draw_features(cv_ptr->image,
                                          msg_ptr->features.positions,
                                          msg_ptr->features.scales,
                                          msg_ptr->features.orientations);
            cv::imshow(_window_name.c_str(), image);
        }
        catch (cv_bridge::Exception error) {
#if ROS_VERSION_MAJOR == 1
            ROS_WARN("bad frame");
#else
            RCLCPP_WARN(_node->get_logger(), "bad frame");
#endif
            return;
        }
    }
};

#if ROS_VERSION_MAJOR == 1
int main(int argc, char **argv)
{
    ros::init(argc,argv,"feature0d_view");
    if( !ros::master::check() )
        return 1;

    boost::shared_ptr<posedetection_msgs::Feature0DView> node(new posedetection_msgs::Feature0DView());
    ros::spin();
    node.reset();
    return 0;
}
#else
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("feature0d_view");
  auto view = std::make_shared<posedetection_msgs::Feature0DView>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
