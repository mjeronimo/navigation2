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

#ifndef NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

#include <memory>
#include <string>

#include "nav2_tasks/behavior_tree_engine.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

namespace nav2_lifecycle_manager
{

class LifecycleManager : public rclcpp::Node
{
public:
  LifecycleManager();
  ~LifecycleManager();

protected:
  // The ROS node to use when calling lifecycle services
  rclcpp::Node::SharedPtr client_node_;

  // The services provided by this node
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr startup_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr shutdown_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resume_srv_;

  // The callbacks for the services
  void startupCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void shutdownCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void pauseCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void resumeCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  nav2_tasks::BtStatus loadAndExecute(const std::string & parameter_name);

  // The Behavior Tree to be used for the various lifecycle manager operations
  // nav2_tasks::BehaviorTreeEngine bt_;

  // The blackboard that will be provided to the BTs
  BT::Blackboard::Ptr blackboard_;

  // Whether to automatically start up the system
  bool autostart_;
};

}  // namespace nav2_lifecycle_manager

#endif  // NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
