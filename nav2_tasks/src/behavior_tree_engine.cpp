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
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_tasks/bt_conversions.hpp"
#include "nav2_tasks/compute_path_to_pose_action.hpp"
#include "nav2_tasks/follow_path_action.hpp"
#include "nav2_tasks/navigate_to_pose_action.hpp"

using namespace std::chrono_literals;

namespace nav2_tasks
{

BehaviorTreeEngine::BehaviorTreeEngine(nav2_lifecycle::LifecycleNode::SharedPtr node)
: node_(node)
{
}

TaskStatus
BehaviorTreeEngine::run(
  BT::Blackboard::Ptr & blackboard,
  const std::string & behavior_tree_xml,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  // The complete behavior tree that results from parsing the incoming XML. When the tree goes
  // out of scope, all the nodes are destroyed
  BT::Tree tree = BT::buildTreeFromText(factory_, behavior_tree_xml, blackboard);

  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // Loop until something happens with ROS or the node completes w/ success or failure
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree.root_node->executeTick();

    // Check if we've received a cancel message
    if (cancelRequested()) {
	  cancelAllActions(tree.root_node);
      return TaskStatus::CANCELED;
    }

    loopRate.sleep();
  }

  return (result == BT::NodeStatus::SUCCESS) ?
         TaskStatus::SUCCEEDED : TaskStatus::FAILED;
}

TaskStatus
BehaviorTreeEngine::run(
  std::unique_ptr<BT::Tree> & tree,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // Loop until something happens with ROS or the node completes w/ success or failure
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree->root_node->executeTick();

    // Check if we've received a cancel message
    if (cancelRequested()) {
	  cancelAllActions(tree->root_node);
      return TaskStatus::CANCELED;
    }

    loopRate.sleep();
  }

  return (result == BT::NodeStatus::SUCCESS) ?
         TaskStatus::SUCCEEDED : TaskStatus::FAILED;
}

BT::Tree
BehaviorTreeEngine::buildTreeFromText(std::string & xml_string, BT::Blackboard::Ptr blackboard)
{
  return BT::buildTreeFromText(factory_, xml_string, blackboard);
}


}  // namespace nav2_tasks
