#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"

using namespace std::chrono_literals;

class ManagerNode : public rclcpp::Node
{
public:
  ManagerNode()
  : Node("manager_node")
  {
    // Clients for services in problem expert and executor
    problem_expert_client_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    // Subscribe to the actions_hub topic to check the failure
    actions_hub_sub_ = this->create_subscription<plansys2_msgs::msg::ActionExecution>(
      "actions_hub", 10, std::bind(&ManagerNode::actions_hub_callback, this, std::placeholders::_1));

    // Create a timer to periodically check the status
    timer_ = this->create_wall_timer(
      1s, std::bind(&ManagerNode::check_status, this));

    // Save initial state to make sure we can reload upon destruction by action failure
    save_initial_knowledge();
  }

private:
  void actions_hub_callback(const plansys2_msgs::msg::ActionExecution::SharedPtr msg)
  {
    // Check if any action has failed
    if (msg->status == "FAILED")
    {
      RCLCPP_WARN(this->get_logger(), "Action %s has failed!", msg->action.c_str());
      reload_knowledge_and_replan();
    }
    else if (msg->status == "SUCCEEDED")
    {
      // Save knowledge after each successful action
      save_current_knowledge();
    }
  }

  void save_initial_knowledge()
  {
    // Save the initial state of the problem expert
    last_good_instances_ = problem_expert_client_->getInstances();
    last_good_predicates_ = problem_expert_client_->getPredicates();
    last_good_functions_ = problem_expert_client_->getFunctions();
  }

  void save_current_knowledge()
  {
    // Save the current state of the problem expert
    last_good_instances_ = problem_expert_client_->getInstances();
    last_good_predicates_ = problem_expert_client_->getPredicates();
    last_good_functions_ = problem_expert_client_->getFunctions();
  }

  void reload_knowledge_and_replan()
  {
    // Clear current knowledge
    problem_expert_client_->clearKnowledge();

    // Reload last good knowledge
    for (const auto & instance : last_good_instances_)
    {
      problem_expert_client_->addInstance(instance);
    }

    for (const auto & predicate : last_good_predicates_)
    {
      problem_expert_client_->addPredicate(predicate);
    }

    for (const auto & function : last_good_functions_)
    {
      problem_expert_client_->addFunction(function);
    }

    // Request executor to create and execute a new plan
    auto plan_opt = executor_client_->getPlan();
    if (plan_opt)
    {
      if (executor_client_->start_plan_execution(*plan_opt))
      {
        RCLCPP_INFO(this->get_logger(), "New plan execution started successfully.");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to start new plan execution.");
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to retrieve a plan.");
    }
  }

  void check_status()
  {
    // Check the current status and handle accordingly
    auto result = executor_client_->getResult();
    if (result && !result->success)
    {
      RCLCPP_WARN(this->get_logger(), "Plan execution has failed.");
      reload_knowledge_and_replan();
    }
  }

  rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr actions_hub_sub_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_client_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<plansys2::Instance> last_good_instances_;
  std::vector<plansys2::Predicate> last_good_predicates_;
  std::vector<plansys2::Function> last_good_functions_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
