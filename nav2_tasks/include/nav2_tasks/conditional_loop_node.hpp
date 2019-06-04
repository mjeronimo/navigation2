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

#ifndef NAV2_TASKS__CONDITIONAL_LOOP_HPP_
#define NAV2_TASKS__CONDITIONAL_LOOP_HPP_

#include <string>

#include "behaviortree_cpp/decorator_node.h"

namespace nav2_tasks
{

class ConditionalLoop : public BT::DecoratorNode
{
public:
  ConditionalLoop(const std::string & name, const BT::NodeParameters & params)
  : BT::DecoratorNode(name, params)
  {
    std::string target_value = "unknown";
    getParam<std::string>("target_value", target_value);
  }

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static const BT::NodeParameters & requiredNodeParameters()
  {
    static BT::NodeParameters params = {{"target_value", "unknown"}};
    return params;
  }

private:
  BT::NodeStatus tick() override;
};

inline BT::NodeStatus ConditionalLoop::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  /*const BT::NodeStatus child_state =*/ child_node_->executeTick();

  bool initial_pose_received = false;
  blackboard()->get<bool>("initial_pose_received", initial_pose_received);

  if (initial_pose_received) {
    printf("ConditionalLoop: initial_pose_received: %d\n", (int) initial_pose_received);
    return BT::NodeStatus::SUCCESS;
  }

#if 0
  switch (child_state) {
    case BT::NodeStatus::RUNNING:
      return BT::NodeStatus::RUNNING;

    case BT::NodeStatus::SUCCESS:
      child_node_->setStatus(BT::NodeStatus::IDLE);
      return BT::NodeStatus::SUCCESS;

    case BT::NodeStatus::FAILURE:
    default:
      child_node_->setStatus(BT::NodeStatus::IDLE);
      return BT::NodeStatus::FAILURE;
  }

  return status();
#else
  return BT::NodeStatus::RUNNING;
#endif
}

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__CONDITIONAL_LOOP_HPP_
