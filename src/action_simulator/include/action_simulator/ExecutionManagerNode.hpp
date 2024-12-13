#ifndef EXECUTION_MANAGER_NODE_HPP_
#define EXECUTION_MANAGER_NODE_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"
#include "plansys2_msgs/msg/knowledge.hpp"
#include "plansys2_msgs/srv/update_executors.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"

#include "lifecycle_msgs/srv/get_state.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>

namespace action_simulator
{

class ExecutionManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  ExecutionManagerNode();

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

private:
  void setup_knowledge(const std::string &file_path);
  void start_execution();
  // void wait_for_clients();
  // void wait_for_service_and_activation(const std::string &service_name, 
  //                                    const std::string &node_name,
  //                                    std::chrono::seconds timeout);

  void create_and_launch_teams();
  // void save_current_knowledge_to_file(const std::string &filename);
  void reload_knowledge_and_replan();

  // // Callback functions
  // void actions_hub_callback(const plansys2_msgs::msg::ActionExecution::SharedPtr msg);

  // Helper functions
  std::vector<std::string> tokenize(const std::string &str, char delimiter) const;

  // Variables
  std::mutex thread_control_mutex_;
  std::condition_variable thread_control_cv_;
  std::map<std::string, std::thread> executor_threads_;
  bool shutdown_flag_{false};

  // Clients
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;

  // std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  // // Subscribers
  // rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr actions_hub_sub_;
};

}  // namespace action_simulator

#endif  // EXECUTION_MANAGER_NODE_HPP_
