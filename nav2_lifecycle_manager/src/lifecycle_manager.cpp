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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_tasks/behavior_tree_engine.hpp"

using namespace std::placeholders;

namespace nav2_lifecycle_manager
{

static const std::string simple_startup = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <SequenceStar name="root_sequence">
      <Message msg="Starting the system bringup..."/>
      <Message msg="Configuring and activating map_server"/>
      <BringUpNode node_name="map_server"/>
      <Message msg="Configuring and activating amcl"/>
      <BringUpNode node_name="amcl"/>
      <Message msg="Configuring and activating world_model"/>
      <BringUpNode node_name="world_model"/>
      <Message msg="Configuring and activating dwb_controller"/>
      <BringUpNode node_name="dwb_controller"/>
      <Message msg="Configuring and activating navfn_planner"/>
      <BringUpNode node_name="navfn_planner"/>
      <Message msg="Configuring and activating bt_navigator"/>
      <BringUpNode node_name="bt_navigator"/>
      <Message msg="The system is active"/>
    </SequenceStar>
  </BehaviorTree> 
</root>
 )";

static const std::string startup_with_manual_localization = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <SequenceStar name="LocalizationSequence">
      <Message msg="Starting the system bringup..."/>
      <Message msg="Configuring and activating map_server"/>
      <BringUpNode node_name="map_server"/>
      <Message msg="Configuring and activating amcl"/>
      <BringUpNode node_name="amcl"/>
      <Timeout msec="10000">
        <ConditionalLoop target_value="initial_pose_received">
          <Sequence>
            <RateController hz="1.0">
              <Message msg="Waiting for initial pose"/>
            </RateController>
            <IsLocalized/>
          </Sequence>
        </ConditionalLoop>
      </Timeout>
    </SequenceStar>
  </BehaviorTree> 
</root>
 )";

static const std::string shutdown_xml_string = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <SequenceStar name="root_sequence">
      <Message msg="Shutting down the system..."/>
      <Message msg="Deactivating, cleaning up, and shutting down bt_navigator"/>
      <ShutDownNode node_name="bt_navigator"/>
      <Message msg="Deactivating, cleaning up, and shutting down navfn_planner"/>
      <ShutDownNode node_name="navfn_planner"/>
      <Message msg="Deactivating, cleaning up, and shutting down dwb_controller"/>
      <ShutDownNode node_name="dwb_controller"/>
      <Message msg="Deactivating, cleaning up, and shutting down world_model"/>
      <ShutDownNode node_name="world_model"/>
      <Message msg="Deactivating, cleaning up, and shutting down amcl"/>
      <ShutDownNode node_name="amcl"/>
      <Message msg="Deactivating, cleaning up, and shutting down map_server"/>
      <ShutDownNode node_name="map_server"/>
      <Message msg="The system has been successfully shut down"/>
  </SequenceStar>
  </BehaviorTree> 
</root>
 )";

static const std::string pause_xml_string = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <SequenceStar name="root_sequence">
      <Message msg="Pausing the system..."/>
      <Message msg="Pausing bt_navigator"/>
      <PauseNode node_name="bt_navigator"/>
      <Message msg="Pausing navfn_planner"/>
      <PauseNode node_name="navfn_planner"/>
      <Message msg="Pausing dwb_controller"/>
      <PauseNode node_name="dwb_controller"/>
      <Message msg="Pausing world_model"/>
      <PauseNode node_name="world_model"/>
      <Message msg="Pausing down amcl"/>
      <PauseNode node_name="amcl"/>
      <Message msg="Pausing map_server"/>
      <PauseNode node_name="map_server"/>
      <Message msg="The system has been successfully paused"/>
    </SequenceStar>
  </BehaviorTree> 
</root>
 )";

static const std::string resume_xml_string = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <SequenceStar name="root_sequence">
      <Message msg="Resuming the system..."/>
      <Message msg="Configuring and activating map_server"/>
      <BringUpNode node_name="map_server"/>
      <Message msg="Configuring and activating amcl"/>
      <BringUpNode node_name="amcl"/>
      <Message msg="Configuring and activating world_model"/>
      <BringUpNode node_name="world_model"/>
      <Message msg="Configuring and activating dwb_controller"/>
      <BringUpNode node_name="dwb_controller"/>
      <Message msg="Configuring and activating navfn_planner"/>
      <BringUpNode node_name="navfn_planner"/>
      <Message msg="Configuring and activating bt_navigator"/>
      <BringUpNode node_name="bt_navigator"/>
      <Message msg="The system is active"/>
    </SequenceStar>
  </BehaviorTree> 
</root>
 )";

LifecycleManager::LifecycleManager()
: Node("lifecycle_manager")
{
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("autostart", rclcpp::ParameterValue(false));
  get_parameter("autostart", autostart_);

  startup_srv_ = create_service<std_srvs::srv::Empty>("lifecycle_manager/startup",
      std::bind(&LifecycleManager::startupCallback, this, _1, _2, _3));

  shutdown_srv_ = create_service<std_srvs::srv::Empty>("lifecycle_manager/shutdown",
      std::bind(&LifecycleManager::shutdownCallback, this, _1, _2, _3));

  pause_srv_ = create_service<std_srvs::srv::Empty>("lifecycle_manager/pause",
      std::bind(&LifecycleManager::pauseCallback, this, _1, _2, _3));

  resume_srv_ = create_service<std_srvs::srv::Empty>("lifecycle_manager/resume",
      std::bind(&LifecycleManager::resumeCallback, this, _1, _2, _3));

  client_node_ = std::make_shared<rclcpp::Node>("lifecycle_manager_service_client");

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create<BT::BlackboardLocal>();

  // Set a couple values on the blackboard that all of the nodes require
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("node_loop_timeout",  // NOLINT
    std::chrono::milliseconds(100));

  if (autostart_) {
    startup();
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
  auto rc = startup();
  
  // Handle the result
  switch (rc) {
    case nav2_tasks::BtStatus::SUCCEEDED:
      return;

    case nav2_tasks::BtStatus::FAILED:
      //RCLCPP_ERROR(get_logger(), "Mission failed");
      fprintf(stderr, "Startup failed!\n");
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
  shutdown();
}

void
LifecycleManager::pauseCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/)
{
  pause();
}

void
LifecycleManager::resumeCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/)
{
  resume();
}

nav2_tasks::BtStatus
LifecycleManager::startup()
{
  return bt_.run(blackboard_, startup_with_manual_localization);
}

nav2_tasks::BtStatus
LifecycleManager::shutdown()
{
  return bt_.run(blackboard_, shutdown_xml_string);
}

nav2_tasks::BtStatus
LifecycleManager::pause()
{
  return bt_.run(blackboard_, pause_xml_string);
}

nav2_tasks::BtStatus
LifecycleManager::resume()
{
  return bt_.run(blackboard_, resume_xml_string);
}

}  // namespace nav2_lifecycle_manager
