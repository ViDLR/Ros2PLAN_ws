#include "action_simulator/STNControllerNode.hpp"

namespace action_simulator {

STNController::STNController() : rclcpp::Node("STNController_node")
{
    execution_status_pub_ = this->create_publisher<std_msgs::msg::String>("/stn_execution_status", 10);
    start_teams_client_ = this->create_client<plansys2_msgs::srv::StartTeams>("/start_teams");
    stop_teams_client_ = this->create_client<plansys2_msgs::srv::StopTeams>("/stop_teams");

}

void STNController::subscribeToSimulationFeedback(const std::string& robot_name) {
    std::string topic = "/simulation_result_" + robot_name;

    simulation_result_subs_[robot_name] = this->create_subscription<plansys2_msgs::msg::ActionExecutionInfo>(
        topic, 10,
        [this, robot_name](const plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg) {
            if (msg->status != plansys2_msgs::msg::ActionExecutionInfo::EXECUTING) return;

            std::string full_action = msg->action_full_name;
            std::string action_type = msg->action;

            // ✅ Extract suffix safely from message_status
            std::string suffix = "";
            if (msg->message_status.size() > 6 && msg->message_status.substr(0, 6) == "0 none") {
                suffix = msg->message_status.substr(6);  // includes space if needed
                full_action += suffix;
            }

            rclcpp::Time start_time(msg->start_stamp);
            double expected_end = start_time.seconds() + msg->estimated_duration;

            // Update robot state
            RobotLiveState state;
            state.current_action = full_action;
            state.args = msg->arguments;
            state.status = msg->status;
            state.status_stamp = msg->status_stamp;
            state.expected_end_time = expected_end;
            robot_live_states_[robot_name] = state;

            // Update team action map if exists
            std::string matched_team = "";
            for (const auto& [team, action_map] : team_action_tracking_) {
                if (action_map.count(full_action)) {
                    matched_team = team;
                    // RCLCPP_INFO(this->get_logger(),
                    //     "[🔍 ACTION-TEAM MATCH] Action [%s] found in team [%s]",
                    //     full_action.c_str(), team.c_str());

                    team_action_tracking_[team][full_action].expected_end_time = expected_end;
                    robot_live_states_[robot_name].current_action = full_action;
                    break;
                } else {
                    RCLCPP_DEBUG(this->get_logger(),
                        "[👀 NO MATCH] Action [%s] NOT in team [%s]", 
                        full_action.c_str(), team.c_str());
                }
            }
            
            if (!matched_team.empty()) {
                team_action_tracking_[matched_team][full_action].expected_end_time = state.expected_end_time;
            
                RCLCPP_DEBUG(this->get_logger(),
                    "✅ [STN] EXECUTING: [%s] from [%s] found in team [%s] map. Updated end=%.2f",
                    full_action.c_str(), robot_name.c_str(), matched_team.c_str(), state.expected_end_time);
            } else {
                RCLCPP_DEBUG(this->get_logger(),
                    "❌ [STN] EXECUTING: [%s] on [%s] NOT found in ANY team map!",
                    full_action.c_str(), robot_name.c_str());
                RCLCPP_DEBUG(this->get_logger(), "💡 Consider checking if robot-to-team assignment is outdated or missing.");
            }

            // Highlight sample and translate_data actions
            if (action_type == "sample" || action_type == "translate_data") {
                RCLCPP_INFO(this->get_logger(),
                    "📥 [STN] EXECUTING [%s] by [%s] | type: [%s] | start=%.2f | end=%.2f | dur=%.2f",
                    full_action.c_str(), robot_name.c_str(), action_type.c_str(),
                    start_time.seconds(), expected_end, msg->estimated_duration);
            }
        });
}

void STNController::initializePaths(
    const std::vector<plansys2_msgs::msg::Team>& teams, 
    const std::map<std::string, plansys2_msgs::msg::Plan>& plans,
    const std::map<std::string, std::vector<std::string>>& dependencies) 
{
    team_plans_ = plans;
    path_dependencies_ = dependencies;

    for (const auto& [team_name, plan] : team_plans_) {
        std::map<std::string, ActionTrackingInfo> action_map;
        std::map<std::string, int> action_name_count;

        for (const auto& item : plan.items) {
            ActionTrackingInfo info;
            info.planned_start_time = item.time;
            info.planned_duration = item.duration;
            info.expected_end_time = item.time + item.duration;

            std::string cleaned_action = item.action;
            if (!cleaned_action.empty() && cleaned_action.front() == '(' && cleaned_action.back() == ')') {
                cleaned_action = cleaned_action.substr(1, cleaned_action.size() - 2);
            }

            std::istringstream iss(cleaned_action);
            std::string token;
            iss >> token >> info.robot;  // extract robot name from action string

            std::string base_action = cleaned_action;
            int count = ++action_name_count[base_action];

            std::string unique_action_name;
            if (action_name_count[base_action] == 1) {
                // First and only time this action name is seen — no suffix
                unique_action_name = base_action;
            } else {
                // More than one — now we add suffixes retroactively
                if (count == 2) {
                    // We already assigned the base_action without suffix once. We need to go back and rename it to " A"
                    // First fix the existing one in action_map
                    for (auto& [k, v] : action_map) {
                        if (k == base_action) {
                            std::string updated_name = base_action + " A";
                            v.action_name = updated_name;
                            action_map[updated_name] = v;
                            action_map.erase(k);
                            break;
                        }
                    }
                    unique_action_name = base_action + " B";
                } else {
                    unique_action_name = base_action + " " + static_cast<char>('A' + count - 1);
                }
            }

            info.action_name = unique_action_name;
            action_map[unique_action_name] = info;
        }

        for (auto& [sample_name, sample_info] : action_map) {
            if (sample_name.find("sample") == std::string::npos)
                continue;

            std::string sample_site = extractToken(sample_name, "site");
            double sample_start = sample_info.planned_start_time;

            std::string best_match;
            double smallest_gap = std::numeric_limits<double>::max();

            for (const auto& [translate_name, translate_info] : action_map) {
                if (translate_name.find("translate_data") == std::string::npos)
                    continue;

                // RCLCPP_INFO(this->get_logger(), "🔍 Candidate for %s: %s @ [%.2f, %.2f]",
                //     sample_name.c_str(), translate_name.c_str(),
                //     translate_info.planned_start_time,
                //     translate_info.expected_end_time);

                std::string translate_site = extractToken(translate_name, "site");
                double translate_start = translate_info.planned_start_time;
                double translate_end = translate_info.expected_end_time;

                // Same site, translate starts before sample, and overlaps (optional)
                // RCLCPP_INFO(this->get_logger(), "translate_site %s, sample_site %s",
                //         translate_site.c_str(), sample_site.c_str());
                if (translate_site == sample_site) {
                    double gap = sample_start - translate_end;
                    // RCLCPP_INFO(this->get_logger(), "⏱️ Gap between sample %.2f and translate end %.2f: %.2f",
                    //     sample_start, translate_end, gap);
                    if (translate_start <= sample_start && translate_end >= sample_start && gap < smallest_gap){
                        smallest_gap = gap;
                        best_match = translate_name;
                    }
                }
            }

            if (!best_match.empty()) {
                sample_info.depends_on.push_back(best_match);
                // RCLCPP_INFO(this->get_logger(),
                //     "🔗 [sample] %s depends on [translate_data] %s",
                //     sample_name.c_str(), best_match.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "❗ No translate_data match found for sample %s", sample_name.c_str());
            }
        }
        team_action_tracking_[team_name] = action_map;
        
        RCLCPP_INFO(this->get_logger(), "Registered actions and dependencies for %s:", team_name.c_str());
        for (const auto& [action_name, info] : action_map) {
            std::stringstream dep_stream;
            for (const auto& dep : info.depends_on) {
                dep_stream << dep << "";
            }

            RCLCPP_INFO(this->get_logger(),
                " ├─ Action: %-35s | Robot: %-10s | Depends On: [%s]",
                action_name.c_str(),
                info.robot.c_str(),
                dep_stream.str().c_str()
            );
        }
    }

    RCLCPP_INFO(this->get_logger(), "🔄 Received %zu teams, %zu plans, and %zu dependencies.",
                teams.size(), plans.size(), dependencies.size());

    for (const auto& [team, plan] : plans) {
        RCLCPP_INFO(this->get_logger(), "➡️ Team [%s] -> Plan Steps: %zu", team.c_str(), plan.items.size());
    }

    for (const auto& [team, deps] : dependencies) {
        std::string dep_list = deps.empty() ? "None" : "";
        for (const auto& dep : deps) {
            dep_list += dep + " ";
        }
        RCLCPP_INFO(this->get_logger(), "➡️ Team [%s] depends on: %s", team.c_str(), dep_list.c_str());
    }

    for (const auto& team : teams) {
        std::string robot_list;
        for (const auto& robot : team.robots) {
            robot_list += robot + " ";
            robot_to_team_[robot] = team.name;
            subscribeToSimulationFeedback(robot);
            delay_publishers_[robot] = this->create_publisher<plansys2_msgs::msg::Failure>(
                "/failing_actions_" + robot, 10);
            plan_publishers_[robot] = this->create_publisher<plansys2_msgs::msg::Plan>(
                "/plan_" + robot, 10);
        }
        team_to_robots_[team.name] = team.robots;
        RCLCPP_INFO(this->get_logger(), "➡️ Team [%s] has robots: %s", team.name.c_str(), robot_list.c_str());
        team_active_[team.name] = false;
    }

    std::vector<plansys2_msgs::msg::Team> initial_teams;
    for (const auto &team : teams) {
        if (path_dependencies_[team.name].empty()) {
            initial_teams.push_back(team);
        }
    }

    for (const auto& [team, deps] : path_dependencies_) {
        for (const auto& dep : deps) {
            reverse_dependencies_[dep].push_back(team);
        }
    }

    requestTeamCreation(initial_teams);

    for (const auto &team : initial_teams) {
        createExecutorClient(team.name);
        createExecutorCallback(team.name);
    }

    RCLCPP_INFO(this->get_logger(), "✅ Execution clients & callbacks setup completed. Ready to spin.");
}

void STNController::triggerInitialExecutions() {
    if (team_active_.empty()) {
        RCLCPP_WARN(this->get_logger(), "⚠️ No teams initialized. Cannot start execution.");
        return;
    }

    std::vector<std::string> initial_teams;
    for (const auto& [team_name, _] : team_active_) {
        if (path_dependencies_[team_name].empty()) {
            initial_teams.push_back(team_name);
        }
    }

    for (const auto& team_name : initial_teams) {
        RCLCPP_INFO(this->get_logger(), "🚀 Starting execution for team: %s", team_name.c_str());
        startTeamExecution(team_name);
        RCLCPP_INFO(this->get_logger(), "✅ Team [%s] started successfully.", team_name.c_str());
    }
}

std::string STNController::extractToken(const std::string& action_string, const std::string& prefix) {
    std::istringstream iss(action_string);
    std::string token;
    while (iss >> token) {
        if (token.rfind(prefix, 0) == 0) {  // checks if token starts with prefix
            return token;
        }
    }
    return "";  // not found
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

void STNController::sendDelayToRobot(const std::string& team_name, const std::string& robot_name, float delay_time) {
    auto msg = plansys2_msgs::msg::Failure();
    plansys2_msgs::msg::FailureItem item;
    item.action = robot_live_states_[robot_name].current_action;
    item.failuretype = "delay " + std::to_string(delay_time);  // e.g., "delay 7.0"
    msg.items.push_back(item);
    delay_publishers_[robot_name]->publish(msg);

    RCLCPP_INFO(this->get_logger(), "📤 Sent delay %.2fs to %s for [%s] in team %s",
                delay_time, robot_name.c_str(), item.action.c_str(), team_name.c_str());
}

void STNController::startTeamSyncTimer(const std::string& team_name) {
    RCLCPP_INFO(this->get_logger(), "⏱️ Starting sync timer for team '%s'", team_name.c_str());

    team_sync_timers_[team_name] = this->create_wall_timer(
        1s,
        [this, team_name]() {
            if (!team_action_tracking_.count(team_name)) {
                RCLCPP_DEBUG(this->get_logger(), "🚨 No action map found for team [%s]", team_name.c_str());
                return;
            }

            auto& action_map = team_action_tracking_[team_name];
            // if (team_name == "team_0") {
            //     RCLCPP_INFO(this->get_logger(), "[🔍 DEBUG in synctimer] Registered actions and dependencies for team_0:");
            //     for (const auto& [action_name, info] : action_map) {
            //         std::stringstream dep_stream;
            //         for (const auto& dep : info.depends_on) {
            //             dep_stream << dep << " ";
            //         }

            //         RCLCPP_INFO(this->get_logger(),
            //             " ├─ Action: %-35s | Robot: %-10s | Depends On: [%s]",
            //             action_name.c_str(),
            //             info.robot.c_str(),
            //             dep_stream.str().c_str()
            //         );
            //     }
            // }
            bool relevant_sync_found = false;

            

            for (const auto& [sample_action_name, sample_info] : action_map) {
                if (sample_action_name.find("sample") == std::string::npos) continue;

                const std::string& sample_robot = sample_info.robot;
                if (!robot_live_states_.count(sample_robot)) continue;

                const auto& sample_state = robot_live_states_[sample_robot];
                if (sample_state.current_action != sample_action_name) continue;

                if (sample_info.depends_on.empty()) continue;

                for (const std::string& translate_action_name : sample_info.depends_on) {
                    if (!action_map.count(translate_action_name)) continue;

                    const auto& translate_info = action_map.at(translate_action_name);
                    const std::string& translate_robot = translate_info.robot;

                    if (!robot_live_states_.count(translate_robot)) continue;

                    const auto& translate_state = robot_live_states_[translate_robot];

                    if (translate_state.current_action != translate_action_name) continue;

                    // ✅ Both actions currently executing
                    relevant_sync_found = true;

                    double sample_end = sample_state.expected_end_time;
                    double translate_end = translate_state.expected_end_time;
                    double buffer = 1.0;

                    if (sample_end > translate_end) {
                        float delay = static_cast<float>(sample_end - translate_end + buffer);
                        sendDelayToRobot(team_name, translate_robot, delay);

                        double new_start = sample_state.status_stamp.seconds() + delay;
                        double new_end = new_start + sample_info.planned_duration;
                        team_action_tracking_[team_name][sample_action_name].expected_end_time = new_end;

                        RCLCPP_INFO(this->get_logger(),
                            "🔄 DELAY: [%s] (robot %s) delayed by %.2fs to wait for [%s]",
                            sample_action_name.c_str(), sample_robot.c_str(), delay, translate_action_name.c_str());
                    }
                }
            }

            if (!relevant_sync_found) {
                RCLCPP_DEBUG(this->get_logger(), "ℹ️ No currently executing (sample, translate_data) pairs found.");
            }
        });
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

    // auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    // if (result != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(this->get_logger(), "Service call to /start_teams failed or timed out");
    //     return;
    // }

    // auto response = future.get();
    // if (!response->success) {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to start teams: %s", response->message.c_str());
    // } else {
    //     RCLCPP_INFO(this->get_logger(), "Teams successfully started.");
    // }
}


// void STNController::requestTeamCreation(const std::vector<plansys2_msgs::msg::Team>& teams) {
//     if (teams.empty()) {
//         RCLCPP_WARN(this->get_logger(), "No ready teams available for creation. Skipping request.");
//         return;
//     }

//     if (!start_teams_client_->wait_for_service(std::chrono::seconds(5))) {
//         RCLCPP_ERROR(this->get_logger(), "/start_teams service not available");
//         return;
//     }

//     auto request = std::make_shared<plansys2_msgs::srv::StartTeams::Request>();
//     request->teams = teams;

//     auto future = start_teams_client_->async_send_request(request);
//     auto weak_this = weak_from_this();  // capture weak_ptr for safe use in timer

//     // Create a polling timer to check for result
//     auto check_timer = this->create_wall_timer(
//         200ms,  // check every 200ms
//         [future, weak_this, check_timer]() mutable {
//             if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
//                 auto self = weak_this.lock();
//                 if (!self) return;

//                 try {
//                     auto response = future.get();
//                     if (!response->success) {
//                         RCLCPP_ERROR(self->get_logger(), "❌ Failed to start teams: %s", response->message.c_str());
//                     } else {
//                         RCLCPP_INFO(self->get_logger(), "✅ Teams successfully started.");
//                     }
//                 } catch (const std::exception &e) {
//                     RCLCPP_ERROR(self->get_logger(), "🔥 Exception during /start_teams call: %s", e.what());
//                 }

//                 // Stop the timer once done
//                 check_timer->cancel();
//                 check_timer.reset();
//             }
//         });
// }

bool STNController::isPathReady(const std::string& team_name) {
    RCLCPP_INFO(this->get_logger(), "🔍 Checking if team %s is ready...", team_name.c_str());

    if (team_active_[team_name]) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Team %s is already active!", team_name.c_str());
        return false;
    }

    for (const auto& dep_team : path_dependencies_[team_name]) {
        RCLCPP_INFO(this->get_logger(), "⛓ Team %s depends on %s (active = %s)",
            team_name.c_str(), dep_team.c_str(), team_active_[dep_team] ? "true" : "false");

        if (team_active_[dep_team]) {
            RCLCPP_WARN(this->get_logger(), "🔄 Team %s depends on %s, which is still active!", 
                        team_name.c_str(), dep_team.c_str());
            return false;
        }
    }

    RCLCPP_INFO(this->get_logger(), "✅ Team %s is now ready to start!", team_name.c_str());
    return true;
}

// ✅ **Start Execution for Ready Teams**

void STNController::startTeamExecution(const std::string& team_name) {
    auto client = executor_clients_[team_name];
    auto team_plan = team_plans_[team_name];

    auto action_client = client->get_action_client();  // Exposes underlying rclcpp_action client

    if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "❌ Action server for team '%s' not available after timeout. Will retry later.", team_name.c_str());
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "🚀 item from plan:");
    for (const auto& item : team_plan.items) {
        RCLCPP_INFO(this->get_logger(), "  - %f %s %f", item.time, item.action.c_str(), item.duration);
    }

    for (const auto& robot : team_to_robots_[team_name]) {
        if (plan_publishers_.count(robot)) {
            plan_publishers_[robot]->publish(team_plan);
            RCLCPP_INFO(this->get_logger(), "📤 Published plan to robot [%s]", robot.c_str());
        }
    }

    if (client->start_plan_execution(team_plan)) {
        team_active_[team_name] = true;
        startTeamSyncTimer(team_name); 
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
    for (const auto& robot : team_to_robots_[team_name]) {
        simulation_result_subs_.erase(robot);
        delay_publishers_.erase(robot);
        robot_live_states_.erase(robot);
        robot_to_team_.erase(robot);
    }
    team_to_robots_.erase(team_name);
    
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


// Okay all of this lead to a crash, we knows that we can have several teams together in the TLCMN, the only problem is when starting execution in those teams, my guess is that the problem comes from the executor nodes ?
// How can we create a clean and working team gestion from STNcontroller though our TLCMN so that we can create and cancel team without leading to crashes ? we are in ros2 Humble  ? 

void STNController::createExecutorCallback(const std::string &team_name) {
    if (executor_callbacks_.count(team_name) > 0) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Callback for team '%s' already exists. Skipping creation.", team_name.c_str());
        return;
    }

    std::string topic = "/" + team_name + "/remaining_plan";

    RCLCPP_INFO(this->get_logger(), "🛠️ Creating plan completion callback for team '%s' on topic: %s.", team_name.c_str(), topic.c_str());

    auto sub = this->create_subscription<plansys2_msgs::msg::Plan>(
        topic, 10,
        [this, team_name](const plansys2_msgs::msg::Plan::SharedPtr msg) {
            if (msg->items.empty()) {
                RCLCPP_INFO(this->get_logger(), "✅ Team '%s' completed execution (empty remaining plan).", team_name.c_str());

                team_active_[team_name] = false;
                removeExecutorCallback(team_name);
                removeExecutorClient(team_name);
                team_sync_timers_.erase(team_name);

                // 🔻 Stop the team nodes via service
                auto stop_client = this->create_client<plansys2_msgs::srv::StopTeams>("/stop_teams");
                if (!stop_client->wait_for_service(std::chrono::seconds(5))) {
                    RCLCPP_ERROR(this->get_logger(), "❌ /stop_teams service not available for team '%s'", team_name.c_str());
                } else {
                    auto request = std::make_shared<plansys2_msgs::srv::StopTeams::Request>();
                    plansys2_msgs::msg::Team team_entry;
                    team_entry.name = team_name;
                    request->teams.push_back(team_entry);
                    stop_client->async_send_request(request);
                    RCLCPP_INFO(this->get_logger(), "📤 Sent stop request for team '%s'", team_name.c_str());
                }

                std::vector<plansys2_msgs::msg::Team> ready_teams;
                if (reverse_dependencies_.count(team_name) > 0) {
                    for (const auto& dependent_team : reverse_dependencies_[team_name]) {
                        if (isPathReady(dependent_team)) {
                            plansys2_msgs::msg::Team dep_team;
                            dep_team.name = dependent_team;
                            dep_team.robots = team_to_robots_[dependent_team];  // Or whatever map holds this info
                            ready_teams.push_back(dep_team);
                        }
                    }
                }

                if (!ready_teams.empty()) {
                    RCLCPP_INFO(this->get_logger(), "🧩 Ready teams being requested:");
                    
                    requestTeamCreation(ready_teams);

                    for (const auto& team : ready_teams) {
                        createExecutorClient(team.name);
                        createExecutorCallback(team.name);
                    }

                    RCLCPP_WARN(this->get_logger(), "⏳ Waiting 5 seconds before starting teams...");
                    rclcpp::sleep_for(std::chrono::seconds(5));

                    for (const auto& team : ready_teams) {
                        startTeamExecution(team.name);
                    }
                }
            }
        });

    executor_callbacks_[team_name] = sub;
    RCLCPP_INFO(this->get_logger(), "✅ Callback for team '%s' registered successfully.", team_name.c_str());
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
