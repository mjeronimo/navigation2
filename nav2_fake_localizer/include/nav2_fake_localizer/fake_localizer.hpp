/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
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

#ifndef NAV2_FAKE_LOCALIZER__FAKE_LOCALIZER_HPP_
#define NAV2_FAKE_LOCALIZER__FAKE_LOCALIZER_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"

namespace nav2_fake_localizer
{

class FakeLocalizer : public rclcpp::Node
{
public:
  FakeLocalizer();

protected:
  void stuffFilter(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void update(const nav_msgs::msg::Odometry & message);
  void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void poseFailure(std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped> & msg, tf2_ros::FilterFailureReason reason);
  void odomFailure(std::shared_ptr<const nav_msgs::msg::Odometry> & msg , tf2_ros::FilterFailureReason reason);

  // Publishers and subscribers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr stuff_sub_;

  // Messages to be published
  geometry_msgs::msg::PoseArray particle_cloud_;
  geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;

  // Message filters subscribers
  message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> * initial_pose_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> * filter_sub_;

  // Message filters
  tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped> * initial_pose_filter_;
  tf2_ros::MessageFilter<nav_msgs::msg::Odometry> * filter_;

  // Transforms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_server_;
  tf2::Transform offset_tf_;

  // Parameters
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string global_frame_id_;
  double delta_x_;
  double delta_y_;
  double delta_yaw_;
  double transform_tolerance_;
};

}  // namespace nav2_fake_localizer

#endif  // NAV2_FAKE_LOCALIZER__FAKE_LOCALIZER_HPP_
