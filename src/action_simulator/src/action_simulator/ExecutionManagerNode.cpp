#include "action_simulator/ExecutionManagerNode.hpp"

namespace action_simulator {

using namespace std::chrono_literals;
using StartTeams = plansys2_msgs::srv::StartTeams;

ExecutionManagerNode::ExecutionManagerNode() : rclcpp::Node("execution_manager_node")
{
    RCLCPP_INFO(this->get_logger(), "Creating ExecutionManagerNode...");
    world_info_publisher_ = this->create_publisher<plansys2_msgs::msg::WorldInfo>("/world_info", 10);
    ExecutionSequenceFunction();
}

void ExecutionManagerNode::publish_world_info(const std::string &file_path) 
{
        RCLCPP_INFO(this->get_logger(), "Loading WorldInfo from JSON file: %s", file_path.c_str());

        // Read JSON file
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open JSON file: %s", file_path.c_str());
            return;
        }

        nlohmann::json world_data;
        try {
            file >> world_data;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing JSON file: %s", e.what());
            return;
        }
        file.close();

        // Create WorldInfo message
        plansys2_msgs::msg::WorldInfo world_info_msg;

        try {
            // Populate Points
            for (const auto &point : world_data["points"]) {
                plansys2_msgs::msg::Point poi;
                poi.id = point["id"];
                poi.coordinates = {point["coordinates"][0], point["coordinates"][1], point["coordinates"][2]};
                poi.type = point["type"];
                world_info_msg.points.push_back(poi);

                RCLCPP_INFO(this->get_logger(), "Added Point: ID=%s, Type=%s, Coordinates=[%f, %f, %f]",
                            poi.id.c_str(), poi.type.c_str(),
                            poi.coordinates[0], poi.coordinates[1], poi.coordinates[2]);
            }

            // Populate Sites
            for (const auto &site : world_data["sites"]) {
                plansys2_msgs::msg::Site site_msg;
                site_msg.id = site["id"];
                site_msg.points = site["points"].get<std::vector<std::string>>();
                site_msg.size = site["size"];
                world_info_msg.sites.push_back(site_msg);

                RCLCPP_INFO(this->get_logger(), "Added Site: ID=%s, Size=%f, Points=[%s]",
                            site_msg.id.c_str(), site_msg.size,
                            rcpputils::join(site_msg.points, ", ").c_str());
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error creating WorldInfo message: %s", e.what());
            return;
        }

        // Publish the WorldInfo message
        world_info_publisher_->publish(world_info_msg);
        RCLCPP_INFO(this->get_logger(), "WorldInfo message published successfully.");
}

// void ExecutionManagerNode::setup_knowledge(const std::string &file_path)
// {
//     RCLCPP_INFO(this->get_logger(), "Loading knowledge from file: %s", file_path.c_str());

//     problem_client_->clearKnowledge();
//     RCLCPP_INFO(this->get_logger(), "Cleared existing knowledge base.");

//     std::ifstream file(file_path);
//     if (!file.is_open()) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to open knowledge setup file: %s", file_path.c_str());
//         return;
//     }

//     std::string line;
//     while (std::getline(file, line)) {
//         RCLCPP_INFO(this->get_logger(), "Processing line: %s", line.c_str());
//         std::istringstream iss(line);
//         std::string command;
//         iss >> command;

//         if (command == "set") {
//             std::string type;
//             iss >> type;
//             if (type == "instance") {
//                 std::string name, category;
//                 iss >> name >> category;
//                 RCLCPP_INFO(this->get_logger(), "Adding instance: %s, %s", name.c_str(), category.c_str());
//                 problem_client_->addInstance(plansys2::Instance(name, category));
//             } else if (type == "predicate") {
//                 std::string predicate;
//                 std::getline(iss, predicate);
//                 predicate = "(" + predicate.substr(predicate.find('(') + 1);
//                 RCLCPP_INFO(this->get_logger(), "Adding predicate: %s", predicate.c_str());
//                 problem_client_->addPredicate(plansys2::Predicate(predicate));
//             } else if (type == "function") {
//                 std::string function;
//                 std::getline(iss, function);
//                 RCLCPP_INFO(this->get_logger(), "Adding function: %s", function.c_str());
//                 problem_client_->addFunction(plansys2::Function(function));
//             } else if (type == "goal") {
//                 std::string goal;
//                 std::getline(iss, goal);
//                 RCLCPP_INFO(this->get_logger(), "Setting goal: %s", goal.c_str());
//                 problem_client_->setGoal(plansys2::Goal(goal));
//             } else {
//                 RCLCPP_WARN(this->get_logger(), "Unknown command type: %s", type.c_str());
//             }
//         }
//     }

//     file.close();
//     RCLCPP_INFO(this->get_logger(), "Knowledge successfully loaded from %s", file_path.c_str());
// }


// void ExecutionManagerNode::Analyze_Plan(const std::string &file_path)
// {
//     std::vector<plansys2_msgs::msg::Team> ExecutionManagerNode::analyzePlan() {
//     // Placeholder logic for analyzing the plan and allocating teams
//     std::vector<plansys2_msgs::msg::Team> teams;

//     plansys2_msgs::msg::Team team1;
//     team1.name = "team1";
//     team1.robots = {"robot1", "robot2"};
//     teams.push_back(team1);

//     plansys2_msgs::msg::Team team2;
//     team2.name = "team2";
//     team2.robots = {"robot3", "robot4"};
//     teams.push_back(team2);

//     return teams;
//     }
// }


// Callbackcreation and disposition 
void ExecutionManagerNode::createExecutorCallback(const std::string &team_name) {
    if (executor_callbacks_.count(team_name) > 0) {
        RCLCPP_WARN(this->get_logger(), "Callback for team '%s' already exists. Skipping creation.", team_name.c_str());
        return;
    }

    std::string topic = "/" + team_name + "/executor_status";

    executor_callbacks_[team_name] = this->create_subscription<plansys2_msgs::msg::ActionExecution>(
        topic, 10,
        [this, team_name](const plansys2_msgs::msg::ActionExecution::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Executor status update for team '%s': %s", team_name.c_str(), msg->status.c_str());

            // Logic to handle executor status updates for the specific team
            if (msg->status == "COMPLETED") {
                RCLCPP_INFO(this->get_logger(), "Team '%s' completed action '%s' successfully.", team_name.c_str(), msg->action.c_str());
            } else if (msg->status == "FAILED") {
                RCLCPP_ERROR(this->get_logger(), "Team '%s' failed action '%s'.", team_name.c_str(), msg->action.c_str());
                // Optional: Trigger replanning or other failure management
            }
        });

    RCLCPP_INFO(this->get_logger(), "Created callback for team '%s'.", team_name.c_str());
}

void ExecutionManagerNode::removeExecutorCallback(const std::string &team_name) {
    if (executor_callbacks_.count(team_name) == 0) {
        RCLCPP_WARN(this->get_logger(), "Callback for team '%s' does not exist. Skipping removal.", team_name.c_str());
        return;
    }

    executor_callbacks_.erase(team_name);
    RCLCPP_INFO(this->get_logger(), "Removed callback for team '%s'.", team_name.c_str());
}

bool ExecutionManagerNode::hasExecutorCallback(const std::string &team_name) const {
    return executor_callbacks_.count(team_name) > 0;
}

void ExecutionManagerNode::addExecutorCallbacks(const std::vector<plansys2_msgs::msg::Team> &teams) {
    
    for (const auto &team : teams) {
        if (!hasExecutorCallback(team.name)) {
            createExecutorCallback(team.name);  // Create a callback for the team
        } else {
            RCLCPP_INFO(this->get_logger(), "Callback for team '%s' already exists. No action taken.", team.name.c_str());
        }
    }
}

// Executor client creation and disposition 

void ExecutionManagerNode::createExecutorClient(const std::string &team_name) {
    if (executor_clients_.count(team_name) > 0) {
        RCLCPP_WARN(this->get_logger(), "Executor client for team '%s' already exists. Skipping creation.", team_name.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Creating executor client for team '%s'.", team_name.c_str());
    executor_clients_[team_name] = std::make_shared<plansys2::ExecutorClient>(team_name);
    RCLCPP_INFO(this->get_logger(), "Executor client for team '%s' created successfully.", team_name.c_str());
}

void ExecutionManagerNode::removeExecutorClient(const std::string &team_name) {
    if (executor_clients_.count(team_name) == 0) {
        RCLCPP_WARN(this->get_logger(), "Executor client for team '%s' does not exist. Skipping removal.", team_name.c_str());
        return;
    }

    executor_clients_.erase(team_name);
    RCLCPP_INFO(this->get_logger(), "Executor client for team '%s' removed successfully.", team_name.c_str());
}

bool ExecutionManagerNode::hasExecutorClient(const std::string &team_name) const {
    return executor_clients_.count(team_name) > 0;
}

void ExecutionManagerNode::addExecutorClients(const std::vector<plansys2_msgs::msg::Team> &teams) {
    for (const auto &team : teams) {
        if (!hasExecutorClient(team.name)) {
            createExecutorClient(team.name);
        } else {
            RCLCPP_INFO(this->get_logger(), "Executor client for team '%s' already exists. No action taken.", team.name.c_str());
        }
    }
}

void ExecutionManagerNode::removeExecutorClients(const std::vector<std::string> &team_names) {
    for (const auto &team_name : team_names) {
        removeExecutorClient(team_name);
    }
}


// Main ExecutionSequenceFunction
void ExecutionManagerNode::ExecutionSequenceFunction()
{
    RCLCPP_INFO(this->get_logger(), "Starting ExecutionSequenceFunction ...");

    // Initialize clients
    domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();

    // Load the problem from a .pddl file
    std::ifstream problem_file("src/my_examples/plansys2_testexample/pddl/MMtest.pddl");
    if (!problem_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open problem file.");
        throw std::runtime_error("Problem file load failed");
    }
    std::string problem_str((std::istreambuf_iterator<char>(problem_file)),
                            std::istreambuf_iterator<char>());
    problem_file.close();

    problem_client_->addProblem(problem_str);

    RCLCPP_INFO(this->get_logger(), "Activating ExecutionManagerNode...");

    // Fetch and analyze the plan
    auto domain = domain_client_->getDomain();
    auto problem = problem_client_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to fetch plan.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Plan fetched with %lu actions.", plan->items.size());

    // Simulated output of Analyze_Plan (Team and robot repartition)
    // std::vector<plansys2_msgs::msg::Team> teams = Analyze_Plan()

    // Testing for analyze plan teams output
    std::vector<plansys2_msgs::msg::Team> teams;
    plansys2_msgs::msg::Team team1;
    team1.name = "team1";
    team1.robots = {"robot0", "robot1"};
    teams.push_back(team1);
    plansys2_msgs::msg::Team team2;
    team2.name = "team2";
    team2.robots = {"robot2", "robot3"};
    teams.push_back(team2);

    // Call the /start_teams service of TLCMN for starting teams
    RCLCPP_INFO(this->get_logger(), "Calling the team creation client...");
    auto start_teams_client = this->create_client<StartTeams>(
    "/start_teams");

    if (!start_teams_client->wait_for_service(20s)) {
        RCLCPP_ERROR(this->get_logger(), "/start_teams service not available");
        return;
    }

    auto request = std::make_shared<StartTeams::Request>();
    request->teams = teams;

    auto future = start_teams_client->async_send_request(request);

    // Spin until the future is complete
    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Service call to /start_teams failed or timed out");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Service call completed. Fetching response...");
    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start teams: %s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Teams successfully started: %s", response->message.c_str());

    
    
        

    // Create callbacks and executor clients for the teams if they dont exist
    addExecutorCallbacks(teams);
    // addExecutorClients(teams);

    // Feed the teams with the world information (problem information)
    publish_world_info("/home/virgile/PHD/Ros2PLAN_ws/src/my_examples/plansys2_testexample/pddl/world_info.json");

    // Simplify for team1 only
    auto executor_client1 = std::make_shared<plansys2::ExecutorClient>(
  "executor_client_team1", "team1", "executor_team1");


    RCLCPP_INFO(this->get_logger(), "Creating Executor Client for team1...");

    RCLCPP_INFO(this->get_logger(), "Executor Client for '/team1/executor_team1' is ready.");

    // Store in a temporary map or variable
    executor_clients_["team1"] = executor_client1;

    executor_client1 -> start_plan_execution(plan.value());
    // Start executor cients

    // // Start plan execution for each team
    // for (const auto &team : teams) {
    //     if (executor_clients_.count(team.name) > 0) {
    //         auto client = executor_clients_[team.name];

    //         RCLCPP_INFO(this->get_logger(), "Starting execution for team '%s'.", team.name.c_str());

    //         // Log the plan details before starting execution
    //         RCLCPP_INFO(this->get_logger(), "Publishing plan to team '%s':", team.name.c_str());
    //         for (const auto &action : plan.value().items) {
    //             RCLCPP_INFO(this->get_logger(), "Action: [%s] Start: %f Duration: %f",
    //                         action.action.c_str(), action.time, action.duration);
    //         }

    //         // Attempt to start plan execution
    //         if (!client->start_plan_execution(plan.value())) {
    //             RCLCPP_ERROR(this->get_logger(), "Failed to start execution for team '%s'.", team.name.c_str());
    //         }
    //     } else {
    //         RCLCPP_WARN(this->get_logger(), "Executor client for team '%s' does not exist. Cannot start execution.", team.name.c_str());
    //     }
    // }


}




}  // namespace action_simulator
