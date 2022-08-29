// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, JSK Robotics Lab.
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
/*
 * object_detection_aggregator.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <algorithm>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer_client.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <posedetection_msgs/ObjectDetection.h>

namespace posedetection_msgs {

  bool compareMessageByStamp(const posedetection_msgs::ObjectDetection& a,
                             const posedetection_msgs::ObjectDetection& b) {
    return a.header.stamp < b.header.stamp;
  }

  std::string validateFrameId(const std::string& frame_id) {
    for (size_t i = 0; i < frame_id.length(); ++i) {
      if (frame_id[i] != '/') {
        return frame_id.substr(i, frame_id.length() - i);
      }
    }
    return frame_id;
  }

  class ObjectDetectionAggregator {
  public:
    ObjectDetectionAggregator()
      : nh_("")
      , pnh_("~")
      , subscribed_(false) {
      // initialize parameters
      pnh_.param("always_subscribe", always_subscribe_, false);
      pnh_.param("pub_rate", pub_rate_, 1.0);
      pnh_.param("queue_size", queue_size_, 10);
      std::string target_frame_id;
      pnh_.param("target_frame_id", target_frame_id, std::string());
      target_frame_id_ = validateFrameId(target_frame_id);
      if (!target_frame_id_.empty()) {
        ROS_INFO("Target frame id is set: %s", target_frame_id_.c_str());
      }

      // initialize transform listener
      tf_buffer_client_ = boost::make_shared<tf2_ros::BufferClient>("/tf2_buffer_server");
      if (!tf_buffer_client_->waitForServer(ros::Duration(5.0))) {
        ROS_WARN("/tf2_buffer_server not found.");
        tf_buffer_client_.reset();
        tf_buffer_ = boost::make_shared<tf2_ros::Buffer>();
        tf2_ros::TransformListener tf_listener(*tf_buffer_);
      }

      // advertise aggregated topic
      ros::SubscriberStatusCallback connect_cb
        = boost::bind(&ObjectDetectionAggregator::connectionCallback, this, _1);
      pub_ = nh_.advertise<posedetection_msgs::ObjectDetection>(
        "ObjectDetection_agg", 1,
        connect_cb, connect_cb,
        ros::VoidConstPtr(), /* latch= */false);

      // start publish timer
      timer_ = nh_.createTimer(ros::Duration(1.0 / pub_rate_),
                               &ObjectDetectionAggregator::periodicCallback, this);
    }

    void connectionCallback(const ros::SingleSubscriberPublisher&) {
      boost::mutex::scoped_lock lock(mutex_);

      if (always_subscribe_) {
        if (!subscribed_) {
          subscribe();
          subscribed_ = true;
        }
      } else {
        if (!subscribed_ && pub_.getNumSubscribers() > 0) {
          subscribe();
          subscribed_ = true;
        } else if (subscribed_ && pub_.getNumSubscribers() == 0) {
          unsubscribe();
          subscribed_ = false;
        }
      }
    }

    void subscribe() {
      sub_ = nh_.subscribe("ObjectDetection", queue_size_,
                           &ObjectDetectionAggregator::messageCallback, this);
    }

    void unsubscribe() {
      sub_.shutdown();
    }

    void messageCallback(const posedetection_msgs::ObjectDetection::ConstPtr& msg) {
      boost::mutex::scoped_lock lock(mutex_);

      // if ~target_frame_id param is empty,
      // use the header.frame_id of the first message as target frame.
      std::string frame_id = validateFrameId(msg->header.frame_id);
      if (target_frame_id_.empty()) {
        target_frame_id_ = frame_id;
        ROS_INFO("Target frame id is set: %s", target_frame_id_.c_str());
      }

      // transform pose frame to target_frame_id_
      if (target_frame_id_ != frame_id) {
        posedetection_msgs::ObjectDetection transformed_msg;
        transformed_msg.header = msg->header;
        transformed_msg.header.frame_id = target_frame_id_;
        transformed_msg.objects.resize(msg->objects.size());
        try {
          geometry_msgs::PoseStamped pose_in, pose_out;
          pose_in.header = msg->header;
          pose_in.header.frame_id = frame_id;
          for (size_t i = 0; i < msg->objects.size(); ++i) {
            pose_in.pose = msg->objects[i].pose;
            if (tf_buffer_client_) {
              tf_buffer_client_->transform(pose_in, pose_out, target_frame_id_);
            } else {
              tf_buffer_->transform(pose_in, pose_out, target_frame_id_);
            }
            posedetection_msgs::Object6DPose& pose6d = transformed_msg.objects[i];
            pose6d.pose = pose_out.pose;
            pose6d.reliability = msg->objects[i].reliability;
            pose6d.type = msg->objects[i].type;
          }
          messages_.push_back(transformed_msg);
        } catch (tf2::TransformException &err) {
          ROS_ERROR_THROTTLE(1.0, "Failed to transform: %s", err.what());
        }
      } else {
        messages_.push_back(*msg);
      }
    }

    void periodicCallback(const ros::TimerEvent& ev) {
      boost::mutex::scoped_lock lock(mutex_);

      if (messages_.empty()) return;

      // sort by stamp
      std::sort(messages_.begin(), messages_.end(), compareMessageByStamp);

      // aggregate detections
      // - use the latest most reliable pose for each type
      posedetection_msgs::ObjectDetection pub_msg;
      pub_msg.header = messages_[messages_.size()-1].header;
      std::map<std::string, posedetection_msgs::Object6DPose> m;
      for (size_t i = 0; i < messages_.size(); ++i) {
        posedetection_msgs::ObjectDetection& d = messages_[i];
        for (size_t j = 0; j < d.objects.size(); ++j) {
          posedetection_msgs::Object6DPose& p = d.objects[j];
          if (!m.count(p.type) ||
              m[p.type].reliability <= p.reliability) {
            m[p.type] = p;
          }
        }
      }

      for (std::map<std::string, posedetection_msgs::Object6DPose>::iterator it = m.begin();
           it != m.end(); ++it) {
        pub_msg.objects.push_back(it->second);
      }

      pub_.publish(pub_msg);
      messages_.clear();
    }

    // variables
    boost::mutex mutex_;
    ros::NodeHandle nh_, pnh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Timer timer_;
    boost::shared_ptr<tf2_ros::BufferClient> tf_buffer_client_;
    boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    bool subscribed_;
    bool always_subscribe_;
    double pub_rate_;
    int queue_size_;
    std::string target_frame_id_;
    std::vector<posedetection_msgs::ObjectDetection> messages_;

  }; // class
} // namespace


int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection_aggregator");

  posedetection_msgs::ObjectDetectionAggregator agg;

  ros::spin();

  return 0;
}
