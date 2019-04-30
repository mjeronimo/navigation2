// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_CONTROLLER__NAV2_CONTROLLER_CLIENT_HPP_
#define NAV2_CONTROLLER__NAV2_CONTROLLER_CLIENT_HPP_

#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"

namespace nav2_controller
{

class Nav2ControllerClient
{
public:
  Nav2ControllerClient();

  void startup();
  void shutdown();
  void pause();
  void resume();

  void set_initial_pose(double x, double y, double theta);
  bool navigate_to_pose(double x, double y, double theta);

protected:
  using Srv = std_srvs::srv::Empty;

  void callService(rclcpp::Client<Srv>::SharedPtr service_client, const char * service_name);

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<Srv::Request> request_;

  rclcpp::Client<Srv>::SharedPtr startup_client_;
  rclcpp::Client<Srv>::SharedPtr pause_client_;
  rclcpp::Client<Srv>::SharedPtr resume_client_;
  rclcpp::Client<Srv>::SharedPtr shutdown_client_;

  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_action_client_;
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__NAV2_CONTROLLER_CLIENT_HPP_
