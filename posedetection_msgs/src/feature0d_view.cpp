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
#include "posedetection_msgs/feature0d_view.hpp"
#include <rclcpp/rclcpp.hpp>
#include <posedetection_msgs/msg/image_feature0_d.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

namespace posedetection_msgs
{
    Feature0DView::Feature0DView(const rclcpp::NodeOptions & options)
    : Node("feature0d_view", options)
    {
        using std::placeholders::_1; 
        _window_name = declare_parameter("window_name", "ImageFeature0D");
        bool autosize = declare_parameter("autosize", false);

        _sub = create_subscription<posedetection_msgs::msg::ImageFeature0D>(
          "~/ImageFeature0D", 1, std::bind(&Feature0DView::image_cb, this, _1));
	    cv::namedWindow(_window_name.c_str(), autosize ? cv::WINDOW_AUTOSIZE : 0);
	    cv::startWindowThread();
    }

    void Feature0DView::image_cb(
      const posedetection_msgs::msg::ImageFeature0D::ConstSharedPtr msg_ptr)
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
            RCLCPP_WARN(get_logger(), "bad frame");
            return;
        }
    }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(posedetection_msgs::Feature0DView)