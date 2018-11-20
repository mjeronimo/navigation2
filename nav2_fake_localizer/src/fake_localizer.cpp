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

#include "nav2_fake_localizer/fake_localizer.hpp"

#include <string>
#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/parameter_client.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

using namespace std::placeholders;

namespace nav2_fake_localizer
{

FakeLocalizer::FakeLocalizer(void)
: Node("FakeLocalizer")
{
  set_parameters({rclcpp::Parameter("use_sim_time", true)});

  // Get the node parameters
  get_parameter_or("odom_frame_id", odom_frame_id_, std::string("odom"));
  get_parameter_or("base_frame_id", base_frame_id_, std::string("base_link"));
  get_parameter_or("global_frame_id", global_frame_id_, std::string("/map"));
  get_parameter_or("delta_x", delta_x_, 0.0);
  get_parameter_or("delta_y", delta_y_, 0.0);
  get_parameter_or("delta_yaw", delta_yaw_, 0.0);
  get_parameter_or("transform_tolerance", transform_tolerance_, 0.1);

  auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](auto) {});

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_server_ = std::make_shared<tf2_ros::TransformBroadcaster>(temp_node);

  // Create the publishers
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose");
  cloud_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("particlecloud");

  // Create the subscriber
  stuff_sub_ = create_subscription<nav_msgs::msg::Odometry>("base_pose_ground_truth",
      std::bind(&FakeLocalizer::stuffFilter, this, std::placeholders::_1));

  particle_cloud_.header.stamp = now();
  particle_cloud_.header.frame_id = global_frame_id_;
  particle_cloud_.poses.resize(1);

  tf2::Quaternion q;
  q.setEuler(-delta_yaw_, 0, 0);
  offset_tf_ = tf2::Transform(q, tf2::Vector3(-delta_x_, -delta_y_, 0.0));

  // Subscribe to odometry info
  filter_sub_ = new message_filters::Subscriber<nav_msgs::msg::Odometry>();
  filter_sub_->subscribe(temp_node, "tb3/odom");

  // and filter for the base_frame_id
  filter_ = new tf2_ros::MessageFilter<nav_msgs::msg::Odometry>(*tf_buffer_, base_frame_id_, 100,
      temp_node);
  filter_->connectInput(*filter_sub_);
  filter_->registerCallback(&FakeLocalizer::update, this);
  filter_->registerFailureCallback(std::bind(&FakeLocalizer::odomFailure, this, _1));

  // Subscribe to and filter "2D Pose Estimate" from RViz:
  initial_pose_sub_ =
    new message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped>();
  initial_pose_sub_->subscribe(temp_node, "initialpose");

  // and filter for the global_frame_id
  initial_pose_filter_ = new tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped>(
    *tf_buffer_, global_frame_id_, 1, temp_node);
  initial_pose_filter_->connectInput(*initial_pose_sub_);
  initial_pose_filter_->registerCallback(&FakeLocalizer::initialPoseReceived, this);
  initial_pose_filter_->registerFailureCallback(std::bind(&FakeLocalizer::poseFailure, this, _1));
}

void FakeLocalizer::stuffFilter(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  RCLCPP_INFO(get_logger(), "FakeLocalizer::stuffFilter");

  // We have to do this to force the message filter to wait for transforms
  // from odom_frame_id_ to base_frame_id_ to be available at time odom_msg.header.stamp.
  // Really, the base_pose_ground_truth should come in with no frame_id b/c it doesn't
  // make sense

  // TODO(mjeronimo): still have to do this with ROS2?

  auto stuff_msg = std::make_shared<nav_msgs::msg::Odometry>();
  *stuff_msg = *odom_msg;
  stuff_msg->header.frame_id = odom_frame_id_;

  filter_->add(stuff_msg);
}

void FakeLocalizer::update(const nav_msgs::msg::Odometry & message)
{
  RCLCPP_INFO(get_logger(), "FakeLocalizer::update");

  tf2::Transform txi;
  tf2::impl::Converter<true, false>::convert(message.pose.pose, txi);
  txi = offset_tf_ * txi;

  geometry_msgs::msg::TransformStamped odom_to_map;
  try {
    geometry_msgs::msg::TransformStamped txi_inv;
    txi_inv.header.frame_id = base_frame_id_;
    txi_inv.header.stamp = message.header.stamp;
    tf2::impl::Converter<false, true>::convert(txi.inverse(), txi_inv.transform);
    tf_buffer_->transform(txi_inv, odom_to_map, odom_frame_id_);
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Failed to transform to %s from %s: %s\n",
      odom_frame_id_.c_str(), base_frame_id_.c_str(), e.what());
    return;
  }

  rclcpp::Duration duration(transform_tolerance_);

  geometry_msgs::msg::TransformStamped trans;
  trans.header.stamp = duration + message.header.stamp;
  trans.header.frame_id = global_frame_id_;
  trans.child_frame_id = message.header.frame_id;

  tf2::Transform odom_to_map_tf2;
  tf2::impl::Converter<true, false>::convert(odom_to_map.transform, odom_to_map_tf2);

  tf2::Transform odom_to_map_inv = odom_to_map_tf2.inverse();
  tf2::impl::Converter<false, true>::convert(odom_to_map_inv, trans.transform);

  tf_server_->sendTransform(trans);

  tf2::Transform current;
  tf2::impl::Converter<true, false>::convert(message.pose.pose, current);

  // Also apply the offset to the pose
  current = offset_tf_ * current;

  geometry_msgs::msg::Transform current_msg;
  tf2::impl::Converter<false, true>::convert(current, current_msg);

  // Publish localized pose
  current_pose_.header = message.header;
  current_pose_.header.frame_id = global_frame_id_;
  current_pose_.pose.pose.orientation = current_msg.rotation;
  current_pose_.pose.pose.position.x = current_msg.translation.x;
  current_pose_.pose.pose.position.y = current_msg.translation.y;
  current_pose_.pose.pose.position.z = current_msg.translation.z;
  pose_pub_->publish(current_pose_);

  // The particle cloud is the current position. Quite convenient.
  particle_cloud_.header = current_pose_.header;
  particle_cloud_.poses[0] = current_pose_.pose.pose;

  cloud_pub_->publish(particle_cloud_);
}

void FakeLocalizer::initialPoseReceived(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  RCLCPP_INFO(get_logger(), "FakeLocalizer: initialPoseReceived");

  tf2::Transform pose;
  tf2::impl::Converter<true, false>::convert(msg.pose.pose, pose);

  if (msg.header.frame_id != global_frame_id_) {
    RCLCPP_WARN(get_logger(),
      "Frame ID of \"initialpose\" (%s) is different from the global frame %s",
      msg.header.frame_id.c_str(), global_frame_id_.c_str());
  }

  // Set offset so that current pose is set to "initialpose"
  geometry_msgs::msg::TransformStamped baseInMap;
  try {
    // Convert the timestamp to a time_point
    rclcpp::Time timestamp = msg.header.stamp;
    tf2::TimePoint tf2_time(std::chrono::nanoseconds(timestamp.nanoseconds()));
    baseInMap = tf_buffer_->lookupTransform(base_frame_id_, global_frame_id_, tf2_time);
  } catch (tf2::TransformException) {
    RCLCPP_WARN(get_logger(), "Failed to lookup transform!");
    return;
  }

  tf2::Transform baseInMapTf2;
  tf2::impl::Converter<true, false>::convert(baseInMap.transform, baseInMapTf2);
  tf2::Transform delta = pose * baseInMapTf2;
  offset_tf_ = delta * offset_tf_;
}

void FakeLocalizer::poseFailure(
  const message_filters::MessageEvent<const geometry_msgs::msg::PoseWithCovarianceStamped> & evt)
{
  auto msg = evt.getMessage();
  RCLCPP_INFO(get_logger(), "msg: %p", (void *) msg.get());
}

void FakeLocalizer::odomFailure(
  const message_filters::MessageEvent<const nav_msgs::msg::Odometry> & evt)
{
  auto msg = evt.getMessage();
  RCLCPP_INFO(get_logger(), "msg: %p", (void *) msg.get());
}

}  // namespace nav2_fake_localizer
