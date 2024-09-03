#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"
#include "plansys2_msgs/msg/knowledge.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_pddl_parser/Utils.h"
#include <fstream>
#include <sstream>
#include <optional>

using namespace std::chrono_literals;

class ManagerNode : public rclcpp::Node
{
public:
  ManagerNode()
  : rclcpp::Node("manager_node"), saved_actions()
  {
  }

  void init()
  {
    // Clients for services in problem expert, domain, planner and executor
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_client_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    // Initialize problem expert with instances and predicates
    setup_knowledge("src/my_examples/plansys2_testexample/config/initial_setup.txt");
    
    // Subscribe to the actions_hub topic to check the failure
    actions_hub_sub_ = this->create_subscription<plansys2_msgs::msg::ActionExecution>(
      "/actions_hub", 10, std::bind(&ManagerNode::actions_hub_callback, this, std::placeholders::_1));

    action_execution_info_sub_ = this->create_subscription<plansys2_msgs::msg::ActionExecutionInfo>(
      "/action_execution_info", 10, std::bind(&ManagerNode::action_execution_info_callback, this, std::placeholders::_1));

    // Save initial state to make sure we can reload upon destruction by action failure
    save_current_knowledge_to_file("src/my_examples/plansys2_testexample/config/new_state.txt");

    // Start initial plan
    start_plan_execution();
  }

private:
  void setup_knowledge(const std::string &file_path = "src/my_examples/plansys2_testexample/config/initial_setup.txt")
  {
    problem_expert_client_->clearKnowledge();

    std::ifstream file(file_path);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open knowledge setup file: %s", file_path.c_str());
        return;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string command;
        iss >> command;

        if (command == "set")
        {
            std::string type;
            iss >> type;
            if (type == "instance")
            {
                std::string name, category;
                iss >> name >> category;
                problem_expert_client_->addInstance(plansys2::Instance(name, category));
            }
            else if (type == "predicate")
            {
                std::string predicate;
                std::getline(iss, predicate);
                // Ensure that the predicate string is properly formatted
                predicate = "(" + predicate.substr(predicate.find('(') + 1); // Remove leading space
                problem_expert_client_->addPredicate(plansys2::Predicate(predicate));
            }
            else if (type == "goal")
            {
                std::string goal;
                std::getline(iss, goal);
                problem_expert_client_->setGoal(plansys2::Goal(goal));
            }
        }
    }
    file.close();

    RCLCPP_INFO(this->get_logger(), "Knowledge loaded successfully from %s", file_path.c_str());

    std::vector<std::string> robot_names = {"robot1", "robot2"};  // Add more robot names as needed
    if (!problem_expert_client_->subscribe_to_knowledge_topics(robot_names)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to knowledge topics.");
      return;
    }
  }
  
  void start_plan_execution()
  {
    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_client_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value())
    {
      RCLCPP_ERROR(this->get_logger(), "Could not find plan to reach goal %s",
                 parser::pddl::toString(problem_expert_client_->getGoal()).c_str());
      return;
    }

    if (!executor_client_->start_plan_execution(plan.value()))
    {
      RCLCPP_ERROR(this->get_logger(), "Error starting a new plan (first)");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Plan execution started successfully.");
    }
  }

  void actions_hub_callback(const plansys2_msgs::msg::ActionExecution::SharedPtr msg)
  {
  std::string status = msg->status;
  RCLCPP_INFO(this->get_logger(), "Action %s status: %s", msg->action.c_str(), status.c_str());

  // Check if any action has failed
  if (status.find("failed") != std::string::npos)
  {
    RCLCPP_WARN(this->get_logger(), "Action %s has failed!", msg->action.c_str());
    RCLCPP_INFO(this->get_logger(), "The planner will soon be restarted");
    reload_knowledge_and_replan();
  }
  else if (status.find("completed") != std::string::npos)
  {
    RCLCPP_INFO(this->get_logger(), "Action %s has succeeded!", msg->action.c_str());
  }
  }


  void action_execution_info_callback(const plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "Action %s status: %d", msg->action_full_name.c_str(), msg->status);

    if (msg->status == plansys2_msgs::msg::ActionExecutionInfo::FAILED)
    {
        RCLCPP_WARN(this->get_logger(), "Action %s has failed! Replanning will be triggered.", msg->action_full_name.c_str());
        reload_knowledge_and_replan();
    }
    else if (msg->status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED && saved_actions.find(msg->action_full_name) == saved_actions.end())
    {
        RCLCPP_INFO(this->get_logger(), "Saving state for action %s.", msg->action_full_name.c_str());
        save_current_knowledge_to_file("src/my_examples/plansys2_testexample/config/new_state.txt");
        saved_actions.insert(msg->action_full_name);  // Track the action as saved
    }
  }

  void save_current_knowledge_to_file(const std::string &filename)
  {
    std::ofstream file(filename);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file %s for saving current knowledge.", filename.c_str());
        return;
    }

    // Write instances
    auto instances = problem_expert_client_->getInstances();
    for (const auto &instance : instances)
    {
        file << "set instance " << instance.name << " " << instance.type << "\n";
    }

    // Write predicates
    auto predicates = problem_expert_client_->getPredicates();
    for (const auto &predicate : predicates)
    {
        file << "set predicate " << parser::pddl::toString(predicate) << "\n";
    }

    // Write functions (if any, you might not have any in your scenario)
    auto functions = problem_expert_client_->getFunctions();
    for (const auto &function : functions)
    {
        file << "set function " << parser::pddl::toString(function) << "\n";
    }

    // Write goal
    auto goal = problem_expert_client_->getGoal();
    file << "set goal " << parser::pddl::toString(goal) << "\n";

    file.close();
    RCLCPP_INFO(this->get_logger(), "Current knowledge successfully saved to %s", filename.c_str());
  }


  void reload_knowledge_and_replan()
  {
    // Ensure the previous plan has fully finished
    while (executor_client_->execute_and_check_plan())
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for the current plan to finish before replanning...");
        std::this_thread::sleep_for(1s);  // Sleep for a second and check again
    }

    // Assuming robot_names are managed as part of your system's configuration
    std::vector<std::string> robot_names = {"robot1", "robot2"}; // Customize as needed
    
    if (!problem_expert_client_->repairknowledge(robot_names)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to repair knowledge before replanning.");
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "Knowledge successfully repaired.");
      // Save the current state after repairing knowledge
      save_current_knowledge_to_file("src/my_examples/plansys2_testexample/config/new_state.txt");
    }
    
    setup_knowledge("src/my_examples/plansys2_testexample/config/new_state.txt");

    // Request executor to create and execute a new plan
    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_client_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not find plan to reach goal %s",
                     parser::pddl::toString(problem_expert_client_->getGoal()).c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "New plan obtained:");
    for (const auto &item : plan.value().items)
    {
        RCLCPP_INFO(this->get_logger(), "  - %s", item.action.c_str());
    }

    if (!executor_client_->start_plan_execution(plan.value()))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start new plan execution.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "New plan execution started successfully.");
    }
  }

  std::unordered_set<std::string> saved_actions;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr actions_hub_sub_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr action_execution_info_sub_;
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_client_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;

  std::vector<plansys2::Instance> last_good_instances_;
  std::vector<plansys2::Predicate> last_good_predicates_;
  std::vector<plansys2::Function> last_good_functions_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManagerNode>();

  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
