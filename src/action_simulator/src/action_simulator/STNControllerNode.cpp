#include "action_simulator/STNControllerNode.hpp"

namespace action_simulator {

STNController::STNController() : rclcpp::Node("STNController_node")
{
    execution_status_pub_ = this->create_publisher<std_msgs::msg::String>("/stn_execution_status", 10);
    start_teams_client_ = this->create_client<plansys2_msgs::srv::StartTeams>("/start_teams");
}

// void STNController::initializePaths(
//     const std::vector<plansys2_msgs::msg::Team>& teams, 
//     const std::map<std::string, plansys2_msgs::msg::Plan>& plans,
//     const std::map<std::string, std::vector<std::string>>& dependencies) {

//     team_plans_ = plans;
//     path_dependencies_ = dependencies;

//     for (const auto &team : teams) {
//         team_active_[team.name] = false;  // Initialize teams as inactive
//         RCLCPP_INFO(this->get_logger(), "🟢 Team [%s] initialized as inactive", team.name.c_str());
//     }

//     RCLCPP_INFO(this->get_logger(), "🔄 Setting up execution clients & callbacks...");

//     std::vector<plansys2_msgs::msg::Team> initial_teams;
//     for (const auto &team : teams) {
//         if (path_dependencies_[team.name].empty()) {  // ✅ Only teams without dependencies
//             initial_teams.push_back(team);
//         }
//     }

//     requestTeamCreation(initial_teams);

//     for (const auto &team : initial_teams) {
//         createExecutorClient(team.name);
//         createExecutorCallback(team.name);
//     }

//     RCLCPP_INFO(this->get_logger(), "✅ Execution clients & callbacks setup completed. Ready to spin.");
// }

// void STNController::triggerInitialExecutions() {
//     RCLCPP_INFO(this->get_logger(), "🚀 Triggering execution for initial teams...");

//     std::vector<std::string> initial_teams;
//     for (const auto& [team_name, _] : team_active_) {
//         if (path_dependencies_[team_name].empty()) {  // ✅ Teams without dependencies
//             initial_teams.push_back(team_name);
//         }
//     }

//     for (const auto& team_name : initial_teams) {
//         startTeamExecution(team_name);
//     }

//     RCLCPP_INFO(this->get_logger(), "✅ Initial teams started. Execution running.");
// }


void STNController::initializePaths(
    const std::vector<plansys2_msgs::msg::Team>& teams, 
    const std::map<std::string, plansys2_msgs::msg::Plan>& plans,
    const std::map<std::string, std::vector<std::string>>& dependencies) {

    team_plans_ = plans;
    path_dependencies_ = dependencies;

    RCLCPP_INFO(this->get_logger(), "🔄 Received %zu teams, %zu plans, and %zu dependencies.",
                teams.size(), plans.size(), dependencies.size());

    // ✅ Print all received plans
    RCLCPP_INFO(this->get_logger(), "📜 Received Plans:");
    for (const auto& [team, plan] : plans) {
        RCLCPP_INFO(this->get_logger(), "➡️ Team [%s] -> Plan Steps: %zu", team.c_str(), plan.items.size());
    }

    // ✅ Print all received dependencies
    RCLCPP_INFO(this->get_logger(), "🔗 Dependencies:");
    for (const auto& [team, deps] : dependencies) {
        std::string dep_list = deps.empty() ? "None" : "";
        for (const auto& dep : deps) {
            dep_list += dep + " ";
        }
        RCLCPP_INFO(this->get_logger(), "➡️ Team [%s] depends on: %s", team.c_str(), dep_list.c_str());
    }

    // ✅ Print all received teams
    RCLCPP_INFO(this->get_logger(), "👥 Teams:");
    for (const auto& team : teams) {
        std::string robot_list;
        for (const auto& robot : team.robots) {
            robot_list += robot + " ";
        }
        RCLCPP_INFO(this->get_logger(), "➡️ Team [%s] has robots: %s", team.name.c_str(), robot_list.c_str());
    }

    for (const auto &team : teams) {
        team_active_[team.name] = false;  // Initialize teams as inactive
        RCLCPP_INFO(this->get_logger(), "🟢 Team [%s] initialized as inactive", team.name.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "🔄 Setting up execution only for team_0...");

    // ✅ Find `team_0` and its direct dependencies
    std::vector<plansys2_msgs::msg::Team> first_teams;
    for (const auto &team : teams) {
        if (team.name == "team_0" || path_dependencies_[team.name] == std::vector<std::string>{"team_0"}) {
            first_teams.push_back(team);
        }
    }

    if (first_teams.empty()) {
        RCLCPP_WARN(this->get_logger(), "⚠️ No related teams found for team_0. Cannot start execution.");
        return;
    }

    // ✅ Request creation of `team_0`
    requestTeamCreation({first_teams[0]});  // Only `team_0`

    // ✅ Create execution client & callback **only for team_0**
    createExecutorClient("team_0");
    createExecutorCallback("team_0");

    RCLCPP_INFO(this->get_logger(), "✅ Execution setup for team_0 completed. Ready to spin.");
}

void STNController::triggerInitialExecutions() {
    if (team_active_.empty()) {
        RCLCPP_WARN(this->get_logger(), "⚠️ No teams initialized. Cannot start execution.");
        return;
    }

    // ✅ Find the first team with no dependencies
    std::string first_team;
    for (const auto& [team_name, _] : team_active_) {
        if (path_dependencies_[team_name].empty()) { // No dependencies
            first_team = team_name;
            break;
        }
    }

    if (first_team.empty()) {
        RCLCPP_WARN(this->get_logger(), "⚠️ No team found without dependencies. Cannot start execution.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "🚀 Starting execution for first team: %s", first_team.c_str());

    startTeamExecution(first_team);

    RCLCPP_INFO(this->get_logger(), "✅ First team [%s] started successfully.", first_team.c_str());
}

// ✅ **Publish Execution Status to EMN**
void STNController::publishExecutionStatus() {
    std_msgs::msg::String status_msg;
    status_msg.data = "Active Teams: ";
    
    for (const auto& team : team_active_) {
        if (team.second) {
            status_msg.data += team.first + " ";
        }
    }

    execution_status_pub_->publish(status_msg);
}

void STNController::propagateDelay(const std::string& team_name, float delay) {
    RCLCPP_WARN(this->get_logger(), "Propagating delay of %f seconds in team '%s'.", delay, team_name.c_str());

    // for (const auto &dep : path_dependencies_[team_name]) {
    //     if (execution_progress_[dep] < 1.0) { // Only delay teams still executing
    //         site_remaining_time_[dep] += delay;
    //     }
    // }

    // // ✅ Notify affected execution clients
    // for (const auto &dep : path_dependencies_[team_name]) {
    //     if (executor_clients_.count(dep)) {
    //         executor_clients_[dep]->adjust_execution_delay(delay); // Hypothetical method
    //     }
    // }
}

void STNController::handleFailure(const std::string& team_name) {
    RCLCPP_WARN(this->get_logger(), "Handling failure for team '%s'.", team_name.c_str());

    team_active_[team_name] = false; // ✅ Mark as inactive

    // ✅ Check if team can be restarted
    if (isPathReady(team_name)) {
        startTeamExecution(team_name);
    } else {
        std_msgs::msg::String failure_msg;
        failure_msg.data = "Failure detected in team: " + team_name;
        execution_status_pub_->publish(failure_msg);
    }
}


// **Team Creation Request to TLCMN**
void STNController::requestTeamCreation(const std::vector<plansys2_msgs::msg::Team>& teams) {
    if (teams.empty()) {
        RCLCPP_WARN(this->get_logger(), "No ready teams available for creation. Skipping request.");
        return;
    }

    if (!start_teams_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "/start_teams service not available");
        return;
    }

    auto request = std::make_shared<plansys2_msgs::srv::StartTeams::Request>();
    request->teams = teams;

    auto future = start_teams_client_->async_send_request(request);

    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Service call to /start_teams failed or timed out");
        return;
    }

    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start teams: %s", response->message.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Teams successfully started.");
    }
}

bool STNController::isPathReady(const std::string& team_name) {
    RCLCPP_INFO(this->get_logger(), "🔍 Checking if team %s is ready...", team_name.c_str());

    if (team_active_[team_name]) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Team %s is already active!", team_name.c_str());
        return false;
    }

    // ✅ Ensure ALL dependencies are completed before starting execution
    for (const auto& dep_team : path_dependencies_[team_name]) {
        if (team_active_[dep_team]) {
            RCLCPP_WARN(this->get_logger(), "🔄 Team %s depends on %s, which is still active!", 
                        team_name.c_str(), dep_team.c_str());
            return false;
        }
    }

    RCLCPP_INFO(this->get_logger(), "✅ Team %s is now ready to start!", team_name.c_str());
    return true;
}

plansys2_msgs::msg::Plan STNController::getStaticPlan() {
    plansys2_msgs::msg::Plan plan;
    plansys2_msgs::msg::PlanItem item;

    item.time = 0.0;
    item.action = "(takeoff robot1 cpbase)";
    item.duration = 4.0;
    plan.items.push_back(item);

    item.time = 0.0;
    item.action = "(takeoff robot0 cpbase)";
    item.duration = 4.0;
    plan.items.push_back(item);

    item.time = 4.001;
    item.action = "(change_site robot0 base site3 cpbase sp9)";
    item.duration = 295.013;
    plan.items.push_back(item);

    item.time = 4.001;
    item.action = "(change_site robot1 base site3 cpbase sp6)";
    item.duration = 283.606;
    plan.items.push_back(item);

    item.time = 287.608;
    item.action = "(navigation_air robot1 sp6 cpsite3 site3)";
    item.duration = 9.48;
    plan.items.push_back(item);

    item.time = 299.015;
    item.action = "(navigation_air robot0 sp9 cpsite3 site3)";
    item.duration = 4.966;
    plan.items.push_back(item);

    item.time = 303.982;
    item.action = "(observe_2r robot1 robot0 cpsite3 site3)";
    item.duration = 10.0;
    plan.items.push_back(item);

    item.time = 313.983;
    item.action = "(navigation_air robot0 cpsite3 sp9 site3)";
    item.duration = 4.966;
    plan.items.push_back(item);

    item.time = 313.983;
    item.action = "(navigation_air robot1 cpsite3 sp9 site3)";
    item.duration = 4.966;
    plan.items.push_back(item);

    item.time = 318.95;
    item.action = "(switch_airwater robot0 sp9 site3)";
    item.duration = 5.0;
    plan.items.push_back(item);

    item.time = 318.95;
    item.action = "(switch_airwater robot1 sp9 site3)";
    item.duration = 5.0;
    plan.items.push_back(item);

    item.time = 323.951;
    item.action = "(navigation_water robot1 sp9 pp7 site3)";
    item.duration = 25.44;
    plan.items.push_back(item);

    item.time = 334.392;
    item.action = "(translate_data robot0 sp9 site3)";
    item.duration = 45.0;
    plan.items.push_back(item);

    item.time = 349.392;
    item.action = "(sample robot1 pp7 site3)";
    item.duration = 30.0;
    plan.items.push_back(item);

    item.time = 379.393;
    item.action = "(navigation_water robot1 pp7 pp6 site3)";
    item.duration = 17.9;
    plan.items.push_back(item);

    item.time = 382.294;
    item.action = "(translate_data robot0 sp9 site3)";
    item.duration = 45.0;
    plan.items.push_back(item);

    item.time = 397.294;
    item.action = "(sample robot1 pp6 site3)";
    item.duration = 30.0;
    plan.items.push_back(item);

    item.time = 427.295;
    item.action = "(navigation_water robot1 pp6 pp5 site3)";
    item.duration = 47.5;
    plan.items.push_back(item);

    item.time = 459.796;
    item.action = "(translate_data robot0 sp9 site3)";
    item.duration = 45.0;
    plan.items.push_back(item);

    item.time = 474.796;
    item.action = "(sample robot1 pp5 site3)";
    item.duration = 30.0;
    plan.items.push_back(item);

    item.time = 504.797;
    item.action = "(switch_waterair robot0 sp9)";
    item.duration = 8.0;
    plan.items.push_back(item);

    item.time = 504.797;
    item.action = "(navigation_water robot1 pp5 sp9 site3)";
    item.duration = 30.18;
    plan.items.push_back(item);

    item.time = 512.798;
    item.action = "(change_site robot0 site3 site5 sp9 cpsite5)";
    item.duration = 410.146;
    plan.items.push_back(item);

    item.time = 534.978;
    item.action = "(switch_waterair robot1 sp9)";
    item.duration = 8.0;
    plan.items.push_back(item);

    item.time = 542.979;
    item.action = "(change_site robot1 site3 site5 sp9 sp11)";
    item.duration = 394.346;
    plan.items.push_back(item);

    item.time = 922.945;
    item.action = "(observe robot0 cpsite5 site5)";
    item.duration = 26.666;
    plan.items.push_back(item);

    item.time = 937.326;
    item.action = "(navigation_air robot1 sp11 sp13 site5)";
    item.duration = 23.313;
    plan.items.push_back(item);

    item.time = 949.612;
    item.action = "(navigation_air robot0 cpsite5 sp14 site5)";
    item.duration = 12.846;
    plan.items.push_back(item);

    item.time = 960.64;
    item.action = "(switch_airwater robot1 sp13 site5)";
    item.duration = 5.0;
    plan.items.push_back(item);

    item.time = 962.459;
    item.action = "(navigation_air robot0 sp14 sp12 site5)";
    item.duration = 13.2;
    plan.items.push_back(item);

    item.time = 965.641;
    item.action = "(navigation_water robot1 sp13 pp11 site5)";
    item.duration = 19.38;
    plan.items.push_back(item);

    item.time = 975.66;
    item.action = "(switch_airwater robot0 sp12 site5)";
    item.duration = 5.0;
    plan.items.push_back(item);

    item.time = 980.661;
    item.action = "(translate_data robot0 sp12 site5)";
    item.duration = 45.0;
    plan.items.push_back(item);

    item.time = 985.022;
    item.action = "(sample robot1 pp11 site5)";
    item.duration = 30.0;
    plan.items.push_back(item);

    item.time = 1015.023;
    item.action = "(navigation_water robot1 pp11 pp10 site5)";
    item.duration = 34.04;
    plan.items.push_back(item);

    item.time = 1034.064;
    item.action = "(translate_data robot0 sp12 site5)";
    item.duration = 45.0;
    plan.items.push_back(item);

    item.time = 1049.064;
    item.action = "(sample robot1 pp10 site5)";
    item.duration = 30.0;
    plan.items.push_back(item);

    item.time = 1079.065;
    item.action = "(navigation_water robot1 pp10 pp9 site5)";
    item.duration = 22.6;
    plan.items.push_back(item);

    item.time = 1086.666;
    item.action = "(translate_data robot0 sp12 site5)";
    item.duration = 45.0;
    plan.items.push_back(item);

    item.time = 1101.666;
    item.action = "(sample robot1 pp9 site5)";
    item.duration = 30.0;
    plan.items.push_back(item);

    item.time = 1131.667;
    item.action = "(navigation_water robot1 pp9 pp12 site5)";
    item.duration = 27.46;
    plan.items.push_back(item);

    item.time = 1144.128;
    item.action = "(translate_data robot0 sp12 site5)";
    item.duration = 45.0;
    plan.items.push_back(item);

    item.time = 1159.128;
    item.action = "(sample robot1 pp12 site5)";
    item.duration = 30.0;
    plan.items.push_back(item);

    item.time = 1189.13;
    item.action = "(switch_waterair robot0 sp12)";
    item.duration = 8.0;
    plan.items.push_back(item);

    item.time = 1189.13;
    item.action = "(navigation_water robot1 pp12 sp12 site5)";
    item.duration = 23.36;
    plan.items.push_back(item);

    item.time = 1197.131;
    item.action = "(change_site robot0 site5 base sp12 cpbase)";
    item.duration = 701.58;
    plan.items.push_back(item);

    item.time = 1212.491;
    item.action = "(switch_waterair robot1 sp12)";
    item.duration = 8.0;
    plan.items.push_back(item);

    item.time = 1220.492;
    item.action = "(change_site robot1 site5 base sp12 cpbase)";
    item.duration = 701.58;
    plan.items.push_back(item);

    item.time = 1898.712;
    item.action = "(landing robot0 cpbase)";
    item.duration = 3.0;
    plan.items.push_back(item);

    item.time = 1922.073;
    item.action = "(landing robot1 cpbase)";
    item.duration = 3.0;
    plan.items.push_back(item);

    return plan;
}



// ✅ **Start Execution for Ready Teams**
void STNController::startTeamExecution(const std::string& team_name) {
    
    plansys2_msgs::msg::Plan my_plan = getStaticPlan();

    RCLCPP_INFO(this->get_logger(), "🚀 Fake items for team: %s", team_name.c_str());
    for (const auto& item : my_plan.items) {
        RCLCPP_INFO(this->get_logger(), "  - %f %s %f", item.time, item.action.c_str(), item.duration);
    }
    auto client = executor_clients_[team_name];
    auto team_plan = team_plans_[team_name];

    RCLCPP_INFO(this->get_logger(), "🚀 item from plan:");
    for (const auto& item : team_plan.items) {
        RCLCPP_INFO(this->get_logger(), "  - %f %s %f", item.time, item.action.c_str(), item.duration);
    }

    if (client->start_plan_execution(my_plan)) {
        team_active_[team_name] = true;
    }
}

// **Stop Team Execution**
void STNController::stopTeamExecution(const std::string& team_name) {
    if (!hasExecutorClient(team_name)) {
        RCLCPP_WARN(rclcpp::get_logger("STNController"), "No executor client for '%s'.", team_name.c_str());
        return;
    }

    auto client = executor_clients_[team_name];
    RCLCPP_INFO(rclcpp::get_logger("STNController"), "Stopping execution for team '%s'.", team_name.c_str());

    client->cancel_plan_execution();
    removeExecutorClient(team_name);
}

// Executor client creation and disposition 

void STNController::createExecutorClient(const std::string &team_name) {
    if (executor_clients_.count(team_name) > 0) {
        RCLCPP_WARN(this->get_logger(), "Executor client for team '%s' already exists. Skipping creation.", team_name.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Creating executor client for team '%s'.", team_name.c_str());

    // Corrected: Store the created ExecutorClient in the map
    executor_clients_[team_name] = std::make_shared<plansys2::ExecutorClient>(
        "executor_client_" + team_name,  // Node name
        team_name,                        // Namespace
        "executor_" + team_name           // Executor name
    );

    RCLCPP_INFO(this->get_logger(), "Executor client for team '%s' created successfully.", team_name.c_str());
}


void STNController::removeExecutorClient(const std::string &team_name) {
    if (executor_clients_.count(team_name) == 0) {
        RCLCPP_WARN(this->get_logger(), "Executor client for team '%s' does not exist. Skipping removal.", team_name.c_str());
        return;
    }

    executor_clients_.erase(team_name);
    RCLCPP_INFO(this->get_logger(), "Executor client for team '%s' removed successfully.", team_name.c_str());
}

bool STNController::hasExecutorClient(const std::string &team_name) const {
    return executor_clients_.count(team_name) > 0;
}

void STNController::createExecutorCallback(const std::string &team_name) {
    if (executor_callbacks_.count(team_name) > 0) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Callback for team '%s' already exists. Skipping creation.", team_name.c_str());
        return;
    }

    std::string topic = "/" + team_name + "/executor_status";

    RCLCPP_INFO(this->get_logger(), "🛠️ Creating callback for team '%s' on topic: %s.", team_name.c_str(), topic.c_str());

    executor_callbacks_[team_name] = this->create_subscription<plansys2_msgs::msg::ActionExecution>(
        topic, 10,
        [this, team_name](const plansys2_msgs::msg::ActionExecution::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "📡 Executor status update for team '%s': %s", team_name.c_str(), msg->status.c_str());

            if (msg->status == "COMPLETED") {
                RCLCPP_INFO(this->get_logger(), "✅ Team '%s' completed execution successfully.", team_name.c_str());

                // Mark team as finished
                team_active_[team_name] = false;
                removeExecutorClient(team_name);
                removeExecutorCallback(team_name);

                // ✅ **Identify Next Teams in Execution Graph**
                std::vector<plansys2_msgs::msg::Team> ready_teams;
                if (path_dependencies_.count(team_name) > 0) { // Ensure the team has dependencies listed
                    for (const auto& dependent_team_name : path_dependencies_[team_name]) {
                        if (isPathReady(dependent_team_name)) {
                            plansys2_msgs::msg::Team dependent_team;
                            dependent_team.name = dependent_team_name;
                            // You might need to fetch the actual robot list from `team_plans_` if needed
                            ready_teams.push_back(dependent_team);
                        }
                    }
                }

                if (!ready_teams.empty()) {
                    // ✅ **Request Creation for Next Teams**
                    requestTeamCreation(ready_teams);

                    // ✅ **Start Execution for Ready Teams**
                    for (const auto& team : ready_teams) {
                        createExecutorClient(team.name);
                        createExecutorCallback(team.name);
                        startTeamExecution(team.name);
                    }
                }

            } else if (msg->status == "FAILED") {
                RCLCPP_ERROR(this->get_logger(), "❌ Team '%s' failed execution.", team_name.c_str());
                handleFailure(team_name);
            }
        });

    RCLCPP_INFO(this->get_logger(), "✅ Callback for team '%s' created successfully.", team_name.c_str());
}


void STNController::removeExecutorCallback(const std::string &team_name) {
    if (executor_callbacks_.count(team_name) == 0) {
        RCLCPP_WARN(this->get_logger(), "Callback for team '%s' does not exist. Skipping removal.", team_name.c_str());
        return;
    }

    executor_callbacks_.erase(team_name);
    RCLCPP_INFO(this->get_logger(), "Removed callback for team '%s'.", team_name.c_str());
}

bool STNController::hasExecutorCallback(const std::string &team_name) const {
    return executor_callbacks_.count(team_name) > 0;
}

}  // namespace action_simulator
