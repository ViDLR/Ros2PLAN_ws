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
  : rclcpp::Node("manager_node")
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
    
    knowledge_sub_ = this->create_subscription<plansys2_msgs::msg::Knowledge>(
    "knowledge_updates", 10, std::bind(&ManagerNode::knowledge_callback, this, std::placeholders::_1));

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

  void knowledge_callback(const plansys2_msgs::msg::Knowledge::SharedPtr msg)
  {
    // Clear previous knowledge
    last_good_instances_.clear();
    last_good_predicates_.clear();
    last_good_functions_.clear();

    // Parse instances
    for (const auto &instance_str : msg->instances)
    {
        auto delimiter_pos = instance_str.find(':');
        if (delimiter_pos != std::string::npos)
        {
            std::string instance_name = instance_str.substr(0, delimiter_pos);
            std::string instance_type = instance_str.substr(delimiter_pos + 1);
            last_good_instances_.emplace_back(instance_name, instance_type);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Invalid instance format: %s", instance_str.c_str());
        }
    }

    // Store predicates as strings (since we may only have string representations)
    for (const auto &predicate_str : msg->predicates)
    {
        last_good_predicates_.emplace_back(predicate_str);
    }

    // Store functions as strings (since we may only have string representations)
    for (const auto &function_str : msg->functions)
    {
        last_good_functions_.emplace_back(function_str);
    }

    // Log the received goal
    RCLCPP_INFO(this->get_logger(), "Updated current goal: %s", msg->goal.c_str());

    // Optionally store the goal in a class member variable for later use
    // current_goal_ = msg->goal; // Uncomment if current_goal_ is needed for later processing

    RCLCPP_INFO(this->get_logger(), "Updated manager node with the latest knowledge");
    save_current_knowledge_to_file("src/my_examples/plansys2_testexample/config/new_state.txt");
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

  void print_knowledge_changes()
  {
    auto current_instances = problem_expert_client_->getInstances();
    auto current_predicates = problem_expert_client_->getPredicates();
    auto current_functions = problem_expert_client_->getFunctions();

    // Log all current knowledge
    RCLCPP_INFO(this->get_logger(), "Current Instances:");
    for (const auto &instance : current_instances)
    {
        RCLCPP_INFO(this->get_logger(), "  - %s: %s", instance.name.c_str(), instance.type.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Current Predicates:");
    for (const auto &predicate : current_predicates)
    {
        RCLCPP_INFO(this->get_logger(), "  - %s", parser::pddl::toString(predicate).c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Current Functions:");
    for (const auto &function : current_functions)
    {
        RCLCPP_INFO(this->get_logger(), "  - %s", parser::pddl::toString(function).c_str());
    }

    // Compare instances
    for (const auto &instance : current_instances)
    {
        auto it = std::find_if(last_good_instances_.begin(), last_good_instances_.end(),
                               [&instance](const plansys2::Instance &i) { return i.name == instance.name && i.type == instance.type; });
        if (it == last_good_instances_.end())
        {
            RCLCPP_INFO(this->get_logger(), "New instance added: %s of type %s", instance.name.c_str(), instance.type.c_str());
        }
    }

    for (const auto &instance : last_good_instances_)
    {
        auto it = std::find_if(current_instances.begin(), current_instances.end(),
                               [&instance](const plansys2::Instance &i) { return i.name == instance.name && i.type == instance.type; });
        if (it == current_instances.end())
        {
            RCLCPP_INFO(this->get_logger(), "Instance removed: %s of type %s", instance.name.c_str(), instance.type.c_str());
        }
    }

    // Compare predicates
    for (const auto &predicate : current_predicates)
    {
        auto it = std::find_if(last_good_predicates_.begin(), last_good_predicates_.end(),
                               [&predicate](const plansys2::Predicate &p) { return parser::pddl::toString(p) == parser::pddl::toString(predicate); });
        if (it == last_good_predicates_.end())
        {
            RCLCPP_INFO(this->get_logger(), "New predicate added: %s", parser::pddl::toString(predicate).c_str());
        }
    }

    for (const auto &predicate : last_good_predicates_)
    {
        auto it = std::find_if(current_predicates.begin(), current_predicates.end(),
                               [&predicate](const plansys2::Predicate &p) { return parser::pddl::toString(p) == parser::pddl::toString(predicate); });
        if (it == current_predicates.end())
        {
            RCLCPP_INFO(this->get_logger(), "Predicate removed: %s", parser::pddl::toString(predicate).c_str());
        }
    }

    // Compare functions
    for (const auto &function : current_functions)
    {
        auto it = std::find_if(last_good_functions_.begin(), last_good_functions_.end(),
                               [&function](const plansys2::Function &f) { return parser::pddl::toString(f) == parser::pddl::toString(function); });
        if (it == last_good_functions_.end())
        {
            RCLCPP_INFO(this->get_logger(), "New function added: %s", parser::pddl::toString(function).c_str());
        }
    }

    for (const auto &function : last_good_functions_)
    {
        auto it = std::find_if(current_functions.begin(), current_functions.end(),
                               [&function](const plansys2::Function &f) { return parser::pddl::toString(f) == parser::pddl::toString(function); });
        if (it == current_functions.end())
        {
            RCLCPP_INFO(this->get_logger(), "Function removed: %s", parser::pddl::toString(function).c_str());
        }
    }
  }
  // void save_current_knowledge()
  // {
  //   // Save the current state of the problem expert
  //   last_good_instances_ = problem_expert_client_->getInstances();
  //   last_good_predicates_ = problem_expert_client_->getPredicates();
  //   last_good_functions_ = problem_expert_client_->getFunctions();
  // }


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


  rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr actions_hub_sub_;
  rclcpp::Subscription<plansys2_msgs::msg::Knowledge>::SharedPtr knowledge_sub_;
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_client_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  rclcpp::TimerBase::SharedPtr timer_;

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
