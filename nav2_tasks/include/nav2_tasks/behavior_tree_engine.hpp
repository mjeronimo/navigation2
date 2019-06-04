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

#ifndef NAV2_TASKS__BEHAVIOR_TREE_ENGINE_HPP_
#define NAV2_TASKS__BEHAVIOR_TREE_ENGINE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/blackboard/blackboard_local.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "nav2_util/global_localization_service_client.hpp"

namespace nav2_tasks
{

enum class BtStatus { SUCCEEDED, FAILED, CANCELED };

class BehaviorTreeEngine
{
public:
  BehaviorTreeEngine();
  virtual ~BehaviorTreeEngine() {}

  BtStatus run(
    BT::Blackboard::Ptr & blackboard,
    const std::string & behavior_tree_xml,
    std::function<bool()> cancelRequested = []() -> bool {return false; },
    std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

  BtStatus run(
    std::unique_ptr<BT::Tree> & tree,
    std::function<bool()> cancelRequested = []() -> bool {return false; },
    std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

  BT::Tree buildTreeFromText(std::string & xml_string, BT::Blackboard::Ptr blackboard);

  void haltAllActions(BT::TreeNode * root_node)
  {
    auto visitor = [](BT::TreeNode * node) {
        if (auto action = dynamic_cast<BT::CoroActionNode *>(node)) {
          action->halt();
        }
      };
    BT::applyRecursiveVisitor(root_node, visitor);
  }

  // In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
  void resetTree(BT::TreeNode * root_node)
  {
    auto visitor = [](BT::TreeNode * node) {
        node->setStatus(BT::NodeStatus::IDLE);
      };
    BT::applyRecursiveVisitor(root_node, visitor);
  }

protected:
  // The ROS node to use when calling services
  rclcpp::Node::SharedPtr service_client_node_;

  // Methods used to register as (simple action) BT nodes
  BT::NodeStatus globalLocalizationServiceRequest();
  BT::NodeStatus clearEntirelyCostmapServiceRequest(BT::TreeNode & tree_node);
  BT::NodeStatus bringUpNode(BT::TreeNode & tree_node);
  BT::NodeStatus shutDownNode(BT::TreeNode & tree_node);
  BT::NodeStatus pauseNode(BT::TreeNode & tree_node);
  BT::NodeStatus message(BT::TreeNode & tree_node);
  BT::NodeStatus setCondition(BT::TreeNode & tree_node);

  void registerSimpleActionWithParameters(const std::string& ID,
    const BT::SimpleActionNode::TickFunctor& tick_functor, const BT:: NodeParameters & params);

  // Service clients
  std::unique_ptr<nav2_util::GlobalLocalizationServiceClient> global_localization_client_;

  // The factory that will be used to dynamically construct the behavior tree
  BT::BehaviorTreeFactory factory_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__BEHAVIOR_TREE_ENGINE_HPP_
