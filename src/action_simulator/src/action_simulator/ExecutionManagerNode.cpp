#include "action_simulator/ExecutionManagerNode.hpp"

namespace action_simulator {

ExecutionManagerNode::ExecutionManagerNode(): rclcpp_lifecycle::LifecycleNode("execution_manager_node") {}


using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


CallbackReturnT
ExecutionManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Configuring ExecutionManagerNode...");

    // Initialize clients
    domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();

    
    // Load the problem from a .pddl file
    std::ifstream problem_file("src/my_examples/plansys2_testexample/pddl/MMtest.pddl");
    if (!problem_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open problem file.");
        return CallbackReturn::FAILURE;
    }
    std::string problem_str((std::istreambuf_iterator<char>(problem_file)),
                            std::istreambuf_iterator<char>());
    problem_file.close();

    problem_client_->addProblem(problem_str);

    // Fetch and analyze plan
    auto domain = domain_client_->getDomain();
    // RCLCPP_INFO(this->get_logger(), "Fetched domain: %s", domain.c_str());
    auto problem = problem_client_->getProblem();
    // RCLCPP_INFO(this->get_logger(), "Fetched problem: %s", problem.c_str());
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to fetch plan.");
        return CallbackReturn::FAILURE;
    }
    

    RCLCPP_INFO(this->get_logger(), "Plan fetched with %lu actions.", plan->items.size());

    // Analyze plan and prepare teams
    // analyze_plan(plan.value());
    // setup_team_topics();

    RCLCPP_INFO(this->get_logger(), "ExecutionManagerNode configured successfully.");
    return CallbackReturn::SUCCESS;
}



CallbackReturnT
ExecutionManagerNode::on_activate(const rclcpp_lifecycle::State & )
{
    RCLCPP_INFO(this->get_logger(), "Activating ExecutionManagerNode...");
    // start_execution();
    return CallbackReturn::SUCCESS;
}

CallbackReturnT
ExecutionManagerNode::on_deactivate(const rclcpp_lifecycle::State & )
{
    RCLCPP_INFO(this->get_logger(), "Deactivating ExecutionManagerNode...");
    // stop_executors_and_robots();
    return CallbackReturn::SUCCESS;
}

CallbackReturnT
ExecutionManagerNode::on_shutdown(const rclcpp_lifecycle::State & )
{
    RCLCPP_INFO(this->get_logger(), "Shutting down ExecutionManagerNode...");
    // stop_executors_and_robots();
    // cleanup_resources();
    return CallbackReturn::SUCCESS;
}

// void ExecutionManagerNode::wait_for_clients()
// {
//     const std::chrono::seconds timeout(10);

//     wait_for_service_and_activation("domain_expert/get_state", "domain_expert", timeout);
//     wait_for_service_and_activation("problem_expert/get_state", "problem_expert", timeout);
//     wait_for_service_and_activation("planner/get_state", "planner", timeout);
// }

// void ExecutionManagerNode::wait_for_service_and_activation(
//     const std::string &service_name, 
//     const std::string &node_name,
//     std::chrono::seconds timeout)
// {
//     RCLCPP_INFO(this->get_logger(), "Waiting for service [%s]...", service_name.c_str());

//     auto client = this->create_client<lifecycle_msgs::srv::GetState>(service_name);
//     if (!client->wait_for_service(timeout)) {
//         throw std::runtime_error("Service " + service_name + " is not available.");
//     }

//     RCLCPP_INFO(this->get_logger(), "Service [%s] is now available.", service_name.c_str());

//     // Check lifecycle state
//     auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
//     auto result = client->async_send_request(request);

//     if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
//         rclcpp::FutureReturnCode::SUCCESS)
//     {
//         auto state = result.get()->current_state.label;
//         RCLCPP_INFO(this->get_logger(), "Node [%s] is currently in state: %s", node_name.c_str(), state.c_str());

//         if (state != "active") {
//             throw std::runtime_error("Node " + node_name + " is not in active state.");
//         }
//     } else {
//         throw std::runtime_error("Failed to get state for node " + node_name);
//     }
// }

void ExecutionManagerNode::setup_knowledge(const std::string &file_path)
{
    RCLCPP_INFO(this->get_logger(), "Loading knowledge from file: %s", file_path.c_str());

    problem_client_->clearKnowledge();
    RCLCPP_INFO(this->get_logger(), "Cleared existing knowledge base.");

    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open knowledge setup file: %s", file_path.c_str());
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        RCLCPP_INFO(this->get_logger(), "Processing line: %s", line.c_str());
        std::istringstream iss(line);
        std::string command;
        iss >> command;

        if (command == "set") {
            std::string type;
            iss >> type;
            if (type == "instance") {
                std::string name, category;
                iss >> name >> category;
                RCLCPP_INFO(this->get_logger(), "Adding instance: %s, %s", name.c_str(), category.c_str());
                problem_client_->addInstance(plansys2::Instance(name, category));
            } else if (type == "predicate") {
                std::string predicate;
                std::getline(iss, predicate);
                predicate = "(" + predicate.substr(predicate.find('(') + 1);
                RCLCPP_INFO(this->get_logger(), "Adding predicate: %s", predicate.c_str());
                problem_client_->addPredicate(plansys2::Predicate(predicate));
            } else if (type == "function") {
                std::string function;
                std::getline(iss, function);
                RCLCPP_INFO(this->get_logger(), "Adding function: %s", function.c_str());
                problem_client_->addFunction(plansys2::Function(function));
            } else if (type == "goal") {
                std::string goal;
                std::getline(iss, goal);
                RCLCPP_INFO(this->get_logger(), "Setting goal: %s", goal.c_str());
                problem_client_->setGoal(plansys2::Goal(goal));
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown command type: %s", type.c_str());
            }
        }
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Knowledge successfully loaded from %s", file_path.c_str());
}


void ExecutionManagerNode::start_execution()
{
  auto domain = domain_client_->getDomain();
  auto problem = problem_client_->getProblem();
  auto plan = planner_client_->getPlan(domain, problem);

  if (!plan.has_value())
  {
    RCLCPP_ERROR(this->get_logger(), "Could not find plan to reach goal %s",
                 parser::pddl::toString(problem_client_->getGoal()).c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Plan obtained with %lu actions.", plan->items.size());

  create_and_launch_teams();
}

void ExecutionManagerNode::create_and_launch_teams()
{
  RCLCPP_INFO(this->get_logger(), "Simulating team creation and launching robots...");
  // Add logic here for team creation based on analysis
  // Simulate team creation based on plan analysis
  // std::unordered_map<std::string, std::vector<std::string>> teams = {
  //     {"team1", {"robot0", "robot1"}},
  //     {"team2", {"robot2", "robot3"}}
  // };
}

// void ExecutionManagerNode::stop_executors_and_robots()
// {
//   {
//     std::lock_guard<std::mutex> lock(thread_control_mutex_);
//     shutdown_flag_ = true;
//   }
//   thread_control_cv_.notify_all();

//   for (auto &[namespace_name, thread] : executor_threads_)
//   {
//     if (thread.joinable())
//     {
//       RCLCPP_INFO(this->get_logger(), "Joining executor thread for namespace: %s", namespace_name.c_str());
//       thread.join();
//     }
//   }
//   executor_threads_.clear();
//   RCLCPP_INFO(this->get_logger(), "All executors and robots stopped successfully.");
// }

// void ExecutionManagerNode::cleanup_resources()
// {
//   RCLCPP_INFO(this->get_logger(), "Cleaning up resources...");
//   domain_expert_.reset();
//   planner_client_.reset();
//   problem_client_.reset();
//   executor_client_.reset();
//   // actions_hub_sub_.reset();
//   // stop_executors_and_robots();
// }

// void ExecutionManagerNode::actions_hub_callback(const plansys2_msgs::msg::ActionExecution::SharedPtr msg)
// {
//   RCLCPP_INFO(this->get_logger(), "Received action status: %s", msg->status.c_str());
// }

}  // namespace action_simulator
