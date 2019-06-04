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

#ifndef NAV2_UTIL__MESSAGE_CHECKER_HPP_
#define NAV2_UTIL__MESSAGE_CHECKER_HPP_

#include <atomic>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_util
{

template<typename MessageT>
class MessageChecker: public rclcpp::Node
{
public:
  MessageChecker(const std::string & topic, const rclcpp::QoS & qos = rclcpp::SystemDefaultsQoS())
    : rclcpp::Node(generate_internal_node_name("message_checker"))
  {
    sub_ = create_subscription<MessageT>(
      topic, qos, std::bind(&MessageChecker::messageCallback, this, std::placeholders::_1));
  }

  bool waitForMessage(const std::string & prompt, 
    const std::chrono::seconds timeout = std::chrono::seconds::max(),
    const std::chrono::seconds prompt_interval = std::chrono::seconds(1))
  {
    std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> interval_start = start;

    while (!message_received_) {
      rclcpp::spin_some(this->get_node_base_interface());

      auto now = std::chrono::high_resolution_clock::now();
      auto interval_elapsed = now - interval_start;

      // Now, get that in seconds
      typedef std::chrono::duration<float> float_seconds;
      auto elapsed_seconds = std::chrono::duration_cast<float_seconds>(interval_elapsed);

      if (elapsed_seconds.count() >= prompt_interval.count()) {
        RCLCPP_INFO(get_logger(), prompt.c_str()); 
        interval_start = std::chrono::high_resolution_clock::now();
      }

      auto total_elapsed = now - start;
      auto total_elapsed_seconds = std::chrono::duration_cast<float_seconds>(total_elapsed);

      if (total_elapsed_seconds.count() >= timeout.count()) {
        return false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return true;
  }

  bool messageReceived()
  {
    rclcpp::spin_some(this->get_node_base_interface());
    return message_received_;
  }

private:
  void messageCallback(typename MessageT::SharedPtr)
  {
    message_received_ = true;
  }

  typename rclcpp::Subscription<MessageT>::ConstSharedPtr sub_;
  std::atomic<bool> message_received_{false};
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__MESSAGE_CHECKER_HPP_
