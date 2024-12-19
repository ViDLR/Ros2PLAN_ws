// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "action_simulator/ExecutionManagerNode.hpp"

#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"


int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

  auto execution_manager_node = std::make_shared<action_simulator::ExecutionManagerNode>();
  
  executor.add_node(execution_manager_node->get_node_base_interface());


  std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> manager_nodes;
  manager_nodes["execution_manager_node"] = std::make_shared<plansys2::LifecycleServiceClient>(
    "execution_manager_node_lc_mngr", "execution_manager_node");


  for (auto & manager_node : manager_nodes) {
    manager_node.second->init();
    executor.add_node(manager_node.second);
  }

  std::shared_future<bool> startup_future = std::async(
    std::launch::async,
    std::bind(plansys2::startup_function, manager_nodes, std::chrono::seconds(60)));
  executor.spin_until_future_complete(startup_future);

  if (!startup_future.get()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("emn_bringup"),
      "Failed to start Execution Manager Node!");
    rclcpp::shutdown();
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("multiexec_bringup"), "Starting MultiThreadedExecutor spin.");
  executor.spin();
  rclcpp::shutdown();
  return 0;
}