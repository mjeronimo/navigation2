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

#include <chrono>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/simple_action_client.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "test_msgs/action/fibonacci.hpp"

using Fibonacci = test_msgs::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

using std::placeholders::_1;
using namespace std::chrono_literals;

class FibonacciServerNode : public rclcpp::Node
{
public:
  FibonacciServerNode()
  : rclcpp::Node("test_server_node")
  {
  }
  
  void on_init()
  {
    action_server_ = std::make_shared<nav2_util::SimpleActionServer<Fibonacci>>(
      shared_from_this(), "fibonacci", std::bind(&FibonacciServerNode::execute, this, std::placeholders::_1));
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // The goal may be pre-empted, so keep a pointer to the current goal
    std::shared_ptr<GoalHandle> current_goal_handle = goal_handle;
  
  preempted:
    // Initialize the goal, feedback, and result
    auto goal = current_goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto result = std::make_shared<Fibonacci::Result>();
  
    // Fibonacci-specific initialization
    rclcpp::Rate loop_rate(5);
    auto & sequence = feedback->sequence;
    sequence.push_back(0);
    sequence.push_back(1);
  
    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if this action has been canceled
      if (current_goal_handle->is_canceling()) {
        result->sequence = sequence;
        current_goal_handle->set_canceled(result);
        return;
      }
  
      // Check if we've gotten an new goal, pre-empting the current one
      if (action_server_->update_requested()) {
        current_goal_handle = action_server_->get_updated_goal_handle();
        goto preempted;
      }
  
      // Update the sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
  
      // Publish feedback
      current_goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }
  
    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      current_goal_handle->set_succeeded(result);
    }
  }

private:
  std::shared_ptr<nav2_util::SimpleActionServer<Fibonacci>> action_server_;
};

class RclCppFixture
{
public:
  RclCppFixture() 
  {
    rclcpp::init(0, nullptr);
    server_thread_ = std::make_unique<std::thread>(std::bind(&RclCppFixture::server_thread_func, this));
  }

  ~RclCppFixture()
  {
    rclcpp::shutdown();
    server_thread_->join();
  }

  void server_thread_func()
  {
    auto node = std::make_shared<FibonacciServerNode>();
    node->on_init();
    rclcpp::spin(node->get_node_base_interface());
  }

  std::unique_ptr<std::thread> server_thread_;
};

RclCppFixture g_rclcppfixture;

class FibonacciTestNode : public rclcpp::Node
{
public:
  FibonacciTestNode()
  : rclcpp::Node(nav2_util::generate_internal_node_name("fibonacci_test_node"))
  {
  }

  void on_init()
  {
    action_client_ =
      std::make_shared<nav2_util::SimpleActionClient<Fibonacci>>(shared_from_this(), "fibonacci");
	action_client_->wait_for_server();
  }

  std::shared_ptr<nav2_util::SimpleActionClient<Fibonacci>> action_client_;
};

class FibonacciTest : public ::testing::Test
{
protected:
  std::shared_ptr<FibonacciTestNode> node_;

  virtual void SetUp()
  {
    node_ = std::make_shared<FibonacciTestNode>();
    node_->on_init();
  };

  virtual void TearDown()
  {
  }
};

TEST_F(FibonacciTest, test_goal_and_feedback)
{
  auto goal = Fibonacci::Goal();
  goal.order = 10;

  int sum = 0;
  int feedback_sum = 0;

  auto feedback_callback = [&feedback_sum](
    rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
      feedback_sum += feedback->sequence.back();
    };

  node_->action_client_->send_goal(goal, feedback_callback);

  for (bool done = false; !done; ) {
    auto result = node_->action_client_->wait_for_result(std::chrono::milliseconds(250));
    switch (result) {
      case nav2_util::ActionStatus::SUCCEEDED:
        {
          auto rc = node_->action_client_->get_result();

          for (auto number : rc.response->sequence) {
            sum += number;
          }

          done = true;
          break;
        }

      case nav2_util::ActionStatus::FAILED:
      case nav2_util::ActionStatus::CANCELED:
        done = true;
        break;

      case nav2_util::ActionStatus::RUNNING:
        break;

      default:
        throw std::logic_error("Invalid status value");
    }
  }

  ASSERT_EQ(sum, 143);

  // We should have received some feedback
  ASSERT_GE(feedback_sum, 0);
}

TEST_F(FibonacciTest, test_cancel)
{
  auto goal = Fibonacci::Goal();
  goal.order = 10;

  int sum = 0;
  int feedback_sum = 0;

  auto feedback_callback = [&feedback_sum](
    rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
      feedback_sum += feedback->sequence.back();
    };

  node_->action_client_->send_goal(goal, feedback_callback);

  bool cancel_received = false;

  for (bool done = false; !done; ) {
    auto result = node_->action_client_->wait_for_result(std::chrono::milliseconds(250));
    switch (result) {
      case nav2_util::ActionStatus::SUCCEEDED:
        {
          auto rc = node_->action_client_->get_result();

          for (auto number : rc.response->sequence) {
            sum += number;
          }

          done = true;
          break;
        }

      case nav2_util::ActionStatus::FAILED:
        done = true;
        break;

      case nav2_util::ActionStatus::CANCELED:
        done = true;
        cancel_received = true;
        break;

      case nav2_util::ActionStatus::RUNNING:
        node_->action_client_->cancel();
        break;

      default:
        throw std::logic_error("Invalid status value");
    }
  }

  ASSERT_EQ(cancel_received, true);

  // We should not have SUCCEEDED so the sum is zero
  ASSERT_EQ(sum, 0);
}
