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

#include "nav2_lifecycle_manager/lifecycle_manager.hpp"

#include <fstream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_tasks/behavior_tree_engine.hpp"

using namespace std::placeholders;

namespace nav2_lifecycle_manager
{

LifecycleManager::LifecycleManager()
: Node("lifecycle_manager")
{
  RCLCPP_INFO(get_logger(), "Creating");

  // The lifecycle manager can automatically start the stack, which is the equivalent of invoking
  // the 'startup' service
  declare_parameter("autostart", rclcpp::ParameterValue(false));

  // Each of the lifecycle manager's services has a corresponiding BT XML file, specified by a parameter
  declare_parameter("bt_xml_filename.startup", rclcpp::ParameterValue(std::string("startup.xml")));
  declare_parameter("bt_xml_filename.shutdown", rclcpp::ParameterValue(std::string("shutdown.xml")));
  declare_parameter("bt_xml_filename.pause", rclcpp::ParameterValue(std::string("pause.xml")));
  declare_parameter("bt_xml_filename.resume", rclcpp::ParameterValue(std::string("resume.xml")));

  // Create the services provided by the lifecycle manager
  startup_srv_ = create_service<std_srvs::srv::Empty>("lifecycle_manager/startup",
      std::bind(&LifecycleManager::startupCallback, this, _1, _2, _3));

  shutdown_srv_ = create_service<std_srvs::srv::Empty>("lifecycle_manager/shutdown",
      std::bind(&LifecycleManager::shutdownCallback, this, _1, _2, _3));

  pause_srv_ = create_service<std_srvs::srv::Empty>("lifecycle_manager/pause",
      std::bind(&LifecycleManager::pauseCallback, this, _1, _2, _3));

  resume_srv_ = create_service<std_srvs::srv::Empty>("lifecycle_manager/resume",
      std::bind(&LifecycleManager::resumeCallback, this, _1, _2, _3));

  client_node_ = std::make_shared<rclcpp::Node>("lifecycle_manager_service_client");

  // Create the blackboard that will be shared by all of the nodes in the Behavior Trees
  blackboard_ = BT::Blackboard::create<BT::BlackboardLocal>();

  // Set a couple values on the blackboard that all of the nodes require
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("node_loop_timeout",  // NOLINT
    std::chrono::milliseconds(100));

  // Autostart, if requested via the parameter
  get_parameter("autostart", autostart_);
  if (autostart_) {
    loadAndExecute("bt_xml_filename.startup");
  }
}

LifecycleManager::~LifecycleManager()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

void
LifecycleManager::startupCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/)
{
  switch (loadAndExecute("bt_xml_filename.startup"))
  {
    case nav2_tasks::BtStatus::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Startup succeeded!");
      return;

    case nav2_tasks::BtStatus::FAILED:
      RCLCPP_ERROR(get_logger(), "Startup failed!");
      return;

    case nav2_tasks::BtStatus::CANCELED:
    default:
      throw std::logic_error("Invalid status return from BT");
  }
}

void
LifecycleManager::shutdownCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/)
{
  switch (loadAndExecute("bt_xml_filename.shutdown"))
  {
    case nav2_tasks::BtStatus::SUCCEEDED:
      return;

    case nav2_tasks::BtStatus::FAILED:
      RCLCPP_ERROR(get_logger(), "Startup failed!");
      return;

    case nav2_tasks::BtStatus::CANCELED:
    default:
      throw std::logic_error("Invalid status return from BT");
  }
}

void
LifecycleManager::pauseCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/)
{
  switch (loadAndExecute("bt_xml_filename.pause"))
  {
    case nav2_tasks::BtStatus::SUCCEEDED:
      return;

    case nav2_tasks::BtStatus::FAILED:
      RCLCPP_ERROR(get_logger(), "Startup failed!");
      return;

    case nav2_tasks::BtStatus::CANCELED:
    default:
      throw std::logic_error("Invalid status return from BT");
  }
}

void
LifecycleManager::resumeCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/)
{
  switch (loadAndExecute("bt_xml_filename.resume"))
  {
    case nav2_tasks::BtStatus::SUCCEEDED:
      return;

    case nav2_tasks::BtStatus::FAILED:
      RCLCPP_ERROR(get_logger(), "Startup failed!");
      return;

    case nav2_tasks::BtStatus::CANCELED:
    default:
      throw std::logic_error("Invalid status return from BT");
  }
}

nav2_tasks::BtStatus
LifecycleManager::loadAndExecute(const std::string & parameter_name)
{
  // Get the BT filename to use from the parameter
  std::string xml_filename;
  get_parameter(parameter_name, xml_filename);

  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(xml_filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(get_logger(), "Couldn't open input XML file: %s", xml_filename.c_str());
    return nav2_tasks::BtStatus::FAILED;
  }

  std::string xml_string = std::string(std::istreambuf_iterator<char>(xml_file),
      std::istreambuf_iterator<char>());

  return bt_.run(blackboard_, xml_string);
}

}  // namespace nav2_lifecycle_manager
