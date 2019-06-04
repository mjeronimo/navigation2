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
    getParam<std::string>("key", key_);

    // Convert the XML string param to a boolean
	std::string temp_value;
    getParam<std::string>("value", temp_value);
	target_value_ = (temp_value == "true");
  }

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static const BT::NodeParameters & requiredNodeParameters()
  {
    static BT::NodeParameters params = {{"key", "unknown"}, {"value", "unknown"}};
    return params;
  }

private:
  BT::NodeStatus tick() override;

  std::string key_;
  bool target_value_;
};

inline BT::NodeStatus ConditionalLoop::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  child_node_->executeTick();

  // We're waiting for the value on the blackboard to match the target
  bool current_value = false;
  blackboard()->get<bool>(key_, current_value);

  if (current_value == target_value_) {
     printf("!!!!!!!!!!!!!key: %s\n", key_.c_str());
     printf("!!!!!!!!!!!!!target_value: %d\n", (int) current_value);
     printf("!!!!!!!!!!!!!current_value: %d\n", (int) target_value_);
  }

  return (current_value == target_value_)? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__CONDITIONAL_LOOP_HPP_
