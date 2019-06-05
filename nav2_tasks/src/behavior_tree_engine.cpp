// Copyright (c) 2018 Intel Corporation
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

#include "nav2_tasks/behavior_tree_engine.hpp"

#include <memory>
#include <string>

#include "behaviortree_cpp/blackboard/blackboard_local.h"
#include "nav2_tasks/back_up_action.hpp"
#include "nav2_tasks/bt_action_node.hpp"
#include "nav2_tasks/bt_conversions.hpp"
#include "nav2_tasks/clear_entirely_costmap_service_client.hpp"
#include "nav2_tasks/compute_path_to_pose_action.hpp"
#include "nav2_tasks/conditional_loop_node.hpp"
#include "nav2_tasks/follow_path_action.hpp"
#include "nav2_tasks/goal_reached_condition.hpp"
#include "nav2_tasks/initial_pose_received_condition.hpp"
#include "nav2_tasks/is_localized_condition.hpp"
#include "nav2_tasks/is_stuck_condition.hpp"
#include "nav2_tasks/navigate_to_pose_action.hpp"
#include "nav2_tasks/rate_controller_node.hpp"
#include "nav2_tasks/spin_action.hpp"
#include "nav2_util/lifecycle_service_client.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::Transition;

namespace nav2_tasks
{

BehaviorTreeEngine::BehaviorTreeEngine()
{
  // A node to be used for service calls
  service_client_node_ = nav2_util::generate_internal_node("behavior_tree_engine");

  // Register our custom action nodes so that they can be included in XML description
  factory_.registerNodeType<nav2_tasks::ComputePathToPoseAction>("ComputePathToPose");
  factory_.registerNodeType<nav2_tasks::FollowPathAction>("FollowPath");
  factory_.registerNodeType<nav2_tasks::BackUpAction>("BackUp");
  factory_.registerNodeType<nav2_tasks::SpinAction>("Spin");

  // Register our custom condition nodes
  factory_.registerNodeType<nav2_tasks::IsStuckCondition>("IsStuck");
  factory_.registerNodeType<nav2_tasks::GoalReachedCondition>("GoalReached");
  factory_.registerNodeType<nav2_tasks::InitialPoseReceivedCondition>("InitialPoseReceived");
  factory_.registerNodeType<nav2_tasks::IsLocalizedCondition>("IsLocalized");

  // Register our custom decorator nodes
  factory_.registerNodeType<nav2_tasks::RateController>("RateController");
  factory_.registerNodeType<nav2_tasks::ConditionalLoop>("ConditionalLoop");

  // Register our simple action nodes
  factory_.registerSimpleAction("globalLocalizationServiceRequest",
    std::bind(&BehaviorTreeEngine::globalLocalizationServiceRequest, this));

  factory_.registerSimpleAction("clearEntirelyCostmapServiceRequest",
    std::bind(&BehaviorTreeEngine::clearEntirelyCostmapServiceRequest, this,
    std::placeholders::_1));

  const BT::NodeParameters bringup_node_params {{"node_name", "unknown"}};
  registerSimpleActionWithParameters("StartNode",
    std::bind(&BehaviorTreeEngine::startNode, this, std::placeholders::_1),
    bringup_node_params);

  const BT::NodeParameters shutdown_node_params {{"node_name", "unknown"}};
  registerSimpleActionWithParameters("StopNode",
    std::bind(&BehaviorTreeEngine::stopNode, this, std::placeholders::_1),
    shutdown_node_params);

  const BT::NodeParameters pause_node_params {{"node_name", "unknown"}};
  registerSimpleActionWithParameters("PauseNode",
    std::bind(&BehaviorTreeEngine::pauseNode, this, std::placeholders::_1),
    pause_node_params);

  const BT::NodeParameters message_params {{"msg", "unknown"}};
  registerSimpleActionWithParameters("Message",
    std::bind(&BehaviorTreeEngine::message, this, std::placeholders::_1),
    message_params);

  const BT::NodeParameters set_condition_params {{"key", "unknown"}, {"value", "unknown"}};
  registerSimpleActionWithParameters("SetCondition",
    std::bind(&BehaviorTreeEngine::setCondition, this, std::placeholders::_1),
    set_condition_params);

  // A service client used in one of the simple action nodes
  global_localization_client_ =
    std::make_unique<nav2_util::GlobalLocalizationServiceClient>("bt_navigator");
}

BtStatus
BehaviorTreeEngine::run(
  BT::Blackboard::Ptr & blackboard,
  const std::string & behavior_tree_xml,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  // Parse the input XML and create the corresponding Behavior Tree
  BT::Tree tree = BT::buildTreeFromText(factory_, behavior_tree_xml, blackboard);

  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // Loop until something happens with ROS or the node completes
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree.root_node->executeTick();

    // Check if we've received a cancel message
    if (cancelRequested()) {
      tree.root_node->halt();
      return BtStatus::CANCELED;
    }

    loopRate.sleep();
  }

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BtStatus
BehaviorTreeEngine::run(
  std::unique_ptr<BT::Tree> & tree,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // Loop until something happens with ROS or the node completes
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree->root_node->executeTick();

    // Check if we've received a cancel message
    if (cancelRequested()) {
      tree->root_node->halt();
      return BtStatus::CANCELED;
    }

    loopRate.sleep();
  }

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree
BehaviorTreeEngine::buildTreeFromText(std::string & xml_string, BT::Blackboard::Ptr blackboard)
{
  return BT::buildTreeFromText(factory_, xml_string, blackboard);
}

BT::NodeStatus
BehaviorTreeEngine::globalLocalizationServiceRequest()
{
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto response = std::make_shared<std_srvs::srv::Empty::Response>();

  auto succeeded = global_localization_client_->invoke(request, response);
  return succeeded ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus
BehaviorTreeEngine::clearEntirelyCostmapServiceRequest(
  BT::TreeNode & tree_node)
{
  std::string service_name = "/local_costmap/clear_entirely_local_costmap";
  tree_node.getParam<std::string>("service_name", service_name);

  nav2_tasks::ClearEntirelyCostmapServiceClient clear_entirely_costmap(service_name);
  auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  try {
    clear_entirely_costmap.wait_for_service(std::chrono::seconds(3));
    auto result = clear_entirely_costmap.invoke(request, std::chrono::seconds(3));
    return BT::NodeStatus::SUCCESS;
  } catch (std::runtime_error & e) {
    return BT::NodeStatus::FAILURE;
  }
}

void
BehaviorTreeEngine::registerSimpleActionWithParameters(
  const std::string & ID,
  const BT::SimpleActionNode::TickFunctor & tick_functor,
  const BT::NodeParameters & params)
{
  BT::NodeBuilder builder =
    [tick_functor, ID](const std::string & name, const BT::NodeParameters & params) {
      return std::unique_ptr<BT::TreeNode>(new BT::SimpleActionNode(name, tick_functor, params));
    };

  BT::TreeNodeManifest manifest = {BT::NodeType::ACTION, ID, params};
  factory_.registerBuilder(manifest, builder);
}

BT::NodeStatus
BehaviorTreeEngine::startNode(BT::TreeNode & tree_node)
{
  std::string node_name;
  tree_node.getParam<std::string>("node_name", node_name);

  nav2_util::LifecycleServiceClient lifecycle_client(node_name, service_client_node_);

  if (!lifecycle_client.change_state(Transition::TRANSITION_CONFIGURE)) {
    RCLCPP_ERROR(service_client_node_->get_logger(), "Failed to configure node: %s",
      node_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!lifecycle_client.change_state(Transition::TRANSITION_ACTIVATE)) {
    RCLCPP_ERROR(service_client_node_->get_logger(), "Failed to activate node: %s",
      node_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
BehaviorTreeEngine::stopNode(BT::TreeNode & tree_node)
{
  std::string node_name;
  tree_node.getParam<std::string>("node_name", node_name);

  nav2_util::LifecycleServiceClient lifecycle_client(node_name, service_client_node_);

  if (!lifecycle_client.change_state(Transition::TRANSITION_DEACTIVATE)) {
    RCLCPP_ERROR(service_client_node_->get_logger(), "Failed to deactivate node: %s",
      node_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!lifecycle_client.change_state(Transition::TRANSITION_CLEANUP)) {
    RCLCPP_ERROR(service_client_node_->get_logger(), "Failed to cleanup node: %s",
      node_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!lifecycle_client.change_state(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN)) {
    RCLCPP_ERROR(service_client_node_->get_logger(), "Failed to shutdown node: %s",
      node_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
BehaviorTreeEngine::pauseNode(BT::TreeNode & tree_node)
{
  std::string node_name;
  tree_node.getParam<std::string>("node_name", node_name);

  nav2_util::LifecycleServiceClient lifecycle_client(node_name, service_client_node_);

  if (!lifecycle_client.change_state(Transition::TRANSITION_DEACTIVATE)) {
    RCLCPP_ERROR(service_client_node_->get_logger(), "Failed to deactivate node: %s",
      node_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!lifecycle_client.change_state(Transition::TRANSITION_CLEANUP)) {
    RCLCPP_ERROR(service_client_node_->get_logger(), "Failed to cleanup node: %s",
      node_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

#define ANSI_COLOR_RESET    "\x1b[0m"
#define ANSI_COLOR_BLUE     "\x1b[34m"

BT::NodeStatus
BehaviorTreeEngine::message(BT::TreeNode & tree_node)
{
  std::string msg;
  tree_node.getParam<std::string>("msg", msg);

  RCLCPP_INFO(service_client_node_->get_logger(),
    ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
BehaviorTreeEngine::setCondition(BT::TreeNode & tree_node)
{
  std::string key;
  tree_node.getParam<std::string>("key", key);

  std::string value;
  tree_node.getParam<std::string>("value", value);

  tree_node.blackboard()->template set<bool>(key, (value == "true") ? true : false);  // NOLINT

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_tasks
