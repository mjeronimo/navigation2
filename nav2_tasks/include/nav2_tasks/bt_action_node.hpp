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

#ifndef NAV2_TASKS__BT_ACTION_NODE_HPP_
#define NAV2_TASKS__BT_ACTION_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace nav2_tasks
{

template<class ActionT>
class BtActionNode : public BT::CoroActionNode
{
public:
  explicit BtActionNode(const std::string & action_name)
  : BT::CoroActionNode(action_name), action_name_(action_name)
  {
  }

  BtActionNode(const std::string & action_name, const BT::NodeParameters & params)
  : BT::CoroActionNode(action_name, params), action_name_(action_name)
  {
  }

  BtActionNode() = delete;

  virtual ~BtActionNode()
  {
  }

  // This is a callback from the BT library invoked after the node is created and after the
  // blackboard has been set for the node. It is the first opportunity for the node to access
  // the blackboard. The derived class does not override this method, but overrides onConfigure
  void onInit() final
  {
    // Initialize the input and output messages
    goal_ = typename ActionT::Goal();
    result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

    // Get the required items from the blackboard
    node_ = blackboard()->template get<rclcpp::Node::SharedPtr>("node");
    node_loop_timeout_ =
      blackboard()->template get<std::chrono::milliseconds>("node_loop_timeout");

    // Now that we have the ROS node to use, create the action client for this BT action
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name_);

    // Make sure the server is actually there before continuing
    action_client_->wait_for_action_server();
  }

  // Derived classes can override this method to perform some local initialization such
  // as getting values from the blackboard.
  virtual void onConfigure()
  {
  }

  virtual void onLoopIteration()
  {
  }

  virtual void onSuccess()
  {
  }

  BT::NodeStatus tick() override
  {
    onConfigure(); // TODO: onTick()

    auto future_goal_handle = action_client_->async_send_goal(goal_);
    if (rclcpp::spin_until_future_complete(node_, future_goal_handle) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("send_goal failed");
    }

    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_) {
      throw std::runtime_error("Goal was rejected by the server");
    }

    auto future_result = goal_handle_->async_result();
    rclcpp::executor::FutureReturnCode rc;
    do {
      rc = rclcpp::spin_until_future_complete(node_, future_result, node_loop_timeout_);

      if (rc == rclcpp::executor::FutureReturnCode::TIMEOUT) {
        setStatusRunningAndYield();
        onLoopIteration();			// TODO: onLoopTimeout
      } 

      // if (rc == rclcpp::executor::FutureReturnCode::ABORTED) {
      //   throw std::runtime_error("Get async result failed");
      // }

    } while (rc != rclcpp::executor::FutureReturnCode::SUCCESS);

    result_ = future_result.get();
    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        onSuccess();
        setStatus(BT::NodeStatus::IDLE);
        return BT::NodeStatus::SUCCESS;

      case rclcpp_action::ResultCode::ABORTED:
        setStatus(BT::NodeStatus::IDLE);
        return BT::NodeStatus::FAILURE;

      case rclcpp_action::ResultCode::CANCELED:
        setStatus(BT::NodeStatus::IDLE);
        return BT::NodeStatus::SUCCESS;

      default:
        throw std::logic_error("BtActionNode::Tick: invalid status value");
    }
  }

  void halt() override
  {
    // Shut the node down if it is currently running
    if (status() == BT::NodeStatus::RUNNING) {
      action_client_->async_cancel_goal(goal_handle_);
      auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
      rclcpp::spin_until_future_complete(node_, future_cancel);
    }

    setStatus(BT::NodeStatus::IDLE);
    CoroActionNode::halt();
  }

protected:
  const std::string action_name_;
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

  typename ActionT::Goal goal_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;

  // The timeout value while to use in the tick loop while waiting for
  // a result from the server
  std::chrono::milliseconds node_loop_timeout_;

  // The action server supports a single goal at a time
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__BT_ACTION_NODE_HPP_
