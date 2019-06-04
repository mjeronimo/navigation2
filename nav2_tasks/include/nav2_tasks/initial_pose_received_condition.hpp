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

#ifndef NAV2_TASKS__INITIAL_POSE_RECEIVED_CONDITION_HPP_
#define NAV2_TASKS__INITIAL_POSE_RECEIVED_CONDITION_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_util/message_checker.hpp"

namespace nav2_tasks
{

class InitialPoseReceivedCondition : public BT::ConditionNode
{
public:
  explicit InitialPoseReceivedCondition(const std::string & condition_name)
  : BT::ConditionNode(condition_name), initial_pose_checker_("initialpose")
  {
  }

  InitialPoseReceivedCondition() = delete;

  BT::NodeStatus tick() override
  {
    return initial_pose_checker_.messageReceived() ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

protected:
  nav2_util::MessageChecker<geometry_msgs::msg::PoseWithCovarianceStamped> initial_pose_checker_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__INITIAL_POSE_RECEIVED_CONDITION_HPP_
