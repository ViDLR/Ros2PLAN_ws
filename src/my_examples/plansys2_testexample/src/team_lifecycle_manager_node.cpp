#include "plansys2_testexample/team_lifecycle_manager.hpp"

TeamLifecycleManager::TeamLifecycleManager() : Node("team_lifecycle_manager"), stop_all_flag_(false)
{
    // Create services for starting and stopping teams
    start_teams_service_ = this->create_service<plansys2_msgs::srv::StartTeams>(
        "start_teams",
        std::bind(&TeamLifecycleManager::handleStartTeams, this, std::placeholders::_1, std::placeholders::_2));
    stop_teams_service_ = this->create_service<plansys2_msgs::srv::StopTeams>(
        "stop_teams",
        std::bind(&TeamLifecycleManager::handleStopTeams, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize the shared MultiThreadedExecutor
    shared_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 4);
    spin_thread_ = std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "Spinning shared executor...");
        shared_executor_->spin();
    });
}

TeamLifecycleManager::~TeamLifecycleManager()
{
    stopAllExecutors();
    if (spin_thread_.joinable())
    {
        spin_thread_.join();
    }
}

void TeamLifecycleManager::handleStartTeams(
    const plansys2_msgs::srv::StartTeams::Request::SharedPtr request,
    const plansys2_msgs::srv::StartTeams::Response::SharedPtr response)
{
    // RCLCPP_INFO(this->get_logger(), "StartTeams request received. Number of teams: %zu", request->teams.size());

    for (const auto &team_entry : request->teams)
    {
        const std::string &team_name = team_entry.name;
        const auto &robots = team_entry.robots; // List of robots for this team

        std::lock_guard<std::mutex> lock(thread_mutex_);
        if (team_nodes_.count(team_name) > 0)
        {
            RCLCPP_WARN(this->get_logger(), "Team %s is already running.", team_name.c_str());
            continue;
        }

        // Create and add a new ExecutorNode for the team
        // RCLCPP_INFO(this->get_logger(), "Starting team: %s", team_name.c_str());
        auto team_node = std::make_shared<plansys2::ExecutorNode>("executor_" + team_name, "/" + team_name);
        shared_executor_->add_node(team_node->get_node_base_interface());
        team_nodes_[team_name] = team_node;

        // Add LifecycleServiceClient for the ExecutorNode
        // RCLCPP_INFO(this->get_logger(), "Adding LifecycleServiceClient for team: %s", team_name.c_str());
        auto lifecycle_client = std::make_shared<plansys2::LifecycleServiceClient>(
            "executor_" + team_name + "_lc_mngr", "executor_" + team_name, "/" + team_name);
        lifecycle_client->init();
        shared_executor_->add_node(lifecycle_client);
        lifecycle_clients_[team_name] = lifecycle_client;

        // Trigger lifecycle transitions
        if (!lifecycle_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure LifecycleServiceClient for team: %s", team_name.c_str());
            continue;
        }

        if (!lifecycle_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to activate LifecycleServiceClient for team: %s", team_name.c_str());
            continue;
        }

        std::vector<std::shared_ptr<rclcpp::Node>> normal_robot_nodes;
        std::vector<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>> lifecycle_robot_nodes;

        for (const auto &robot : robots)
        {

            // ✅ Only create simulator if it doesn't already exist
            if (simulator_nodes_.count(robot) == 0)
            {
                auto simulator_node = std::make_shared<SimulationNode>(robot, "");
                simulator_node->set_parameter(rclcpp::Parameter("robot_name", robot));
                simulator_node->set_parameter(rclcpp::Parameter("team_name", team_name));

                shared_executor_->add_node(simulator_node);
                simulator_nodes_[robot] = simulator_node;  // Store it globally
                // RCLCPP_INFO(this->get_logger(), "Created persistent simulator for robot: %s", robot.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Simulator for robot %s already exists, skipping creation.", robot.c_str());
            }

            for (const auto &action_type : getAllActionTypes())
            {
                auto action_node = createActionNode(action_type, robot, team_name);
                action_node->set_parameter(rclcpp::Parameter("action_name", action_type));
                action_node->set_parameter(rclcpp::Parameter("specialized_arguments", std::vector<std::string>{robot}));
                action_node->set_parameter(rclcpp::Parameter("team_name", team_name));

                action_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
                shared_executor_->add_node(action_node->get_node_base_interface());
                lifecycle_robot_nodes.push_back(action_node);
            }

        }

        normal_robot_nodes_[team_name] = normal_robot_nodes;
        lifecycle_robot_nodes_[team_name] = lifecycle_robot_nodes;
        RCLCPP_INFO(this->get_logger(), "Team %s started with %lu robots.", team_name.c_str(), robots.size());
    }

    response->success = true;
    response->message = "Teams started successfully.";
    
    // RCLCPP_INFO(this->get_logger(), "Responding to StartTeams with success: %d, message: %s",
    //         response->success, response->message.c_str());
    
}

// void TeamLifecycleManager::handleStopTeams(
//     const plansys2_msgs::srv::StopTeams::Request::SharedPtr request,
//     const plansys2_msgs::srv::StopTeams::Response::SharedPtr response) {

//     std::lock_guard<std::mutex> lock(thread_mutex_);
//     for (const auto &team_entry : request->teams) {
//         const std::string &team_name = team_entry.name;

//         if (team_nodes_.count(team_name) == 0) {
//             RCLCPP_WARN(this->get_logger(), "Team %s is not running.", team_name.c_str());
//             continue;
//         }

//         // Stop normal nodes
//         if (normal_robot_nodes_.count(team_name) > 0) {
//             RCLCPP_INFO(this->get_logger(), "Stopping normal nodes for team: %s", team_name.c_str());
//             for (auto &node : normal_robot_nodes_[team_name]) {
//                 shared_executor_->remove_node(node);
//             }
//             normal_robot_nodes_.erase(team_name);
//         }

//         // Stop lifecycle nodes
//         if (lifecycle_robot_nodes_.count(team_name) > 0) {
//             RCLCPP_INFO(this->get_logger(), "Stopping lifecycle nodes for team: %s", team_name.c_str());
//             for (auto &node : lifecycle_robot_nodes_[team_name]) {
//                 auto current_state = node->get_current_state().id();

//                 if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
//                     RCLCPP_INFO(this->get_logger(), "Deactivating node: %s", node->get_name());
//                     node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
//                 }

//                 current_state = node->get_current_state().id();
//                 if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
//                     RCLCPP_INFO(this->get_logger(), "Shutting down node: %s", node->get_name());
//                     node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
//                 } else {
//                     RCLCPP_WARN(this->get_logger(), "Node %s is in unexpected state %u, skipping shutdown.", 
//                                 node->get_name(), current_state);
//                 }

//                 shared_executor_->remove_node(node->get_node_base_interface());
//             }
//             lifecycle_robot_nodes_.erase(team_name);
//         }

//         // Deactivate and remove executor lifecycle manager
//         if (lifecycle_clients_.count(team_name) > 0) {
//             auto lifecycle_client = lifecycle_clients_[team_name];
//             RCLCPP_INFO(this->get_logger(), "Shutting down LifecycleServiceClient for team: %s", team_name.c_str());
//             if (!lifecycle_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
//                 RCLCPP_WARN(this->get_logger(), "Failed to deactivate LifecycleServiceClient for team: %s", team_name.c_str());
//             }
//             if (!lifecycle_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN)) {
//                 RCLCPP_WARN(this->get_logger(), "Failed to shutdown LifecycleServiceClient for team: %s", team_name.c_str());
//             }
//             shared_executor_->remove_node(lifecycle_client);
//             lifecycle_clients_.erase(team_name);
//         }

//         // Remove executor node
//         auto executor_node = team_nodes_[team_name];
//         RCLCPP_INFO(this->get_logger(), "Removing executor node for team: %s", team_name.c_str());
//         shared_executor_->remove_node(executor_node->get_node_base_interface());
//         team_nodes_.erase(team_name);
//     }

//     response->success = true;
//     response->message = "Teams stopped successfully.";
// }

void TeamLifecycleManager::handleStopTeams(
    const plansys2_msgs::srv::StopTeams::Request::SharedPtr request,
    const plansys2_msgs::srv::StopTeams::Response::SharedPtr response) {

    std::lock_guard<std::mutex> lock(thread_mutex_);
    for (const auto &team_entry : request->teams) {
        const std::string &team_name = team_entry.name;

        if (team_nodes_.count(team_name) == 0) {
            RCLCPP_WARN(this->get_logger(), "Team %s is not running.", team_name.c_str());
            continue;
        }

        // ✅ Stop action nodes but NOT simulators
        if (lifecycle_robot_nodes_.count(team_name) > 0) {
            for (auto &node : lifecycle_robot_nodes_[team_name]) {
                auto current_state = node->get_current_state().id();

                if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
                }

                current_state = node->get_current_state().id();
                if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
                }

                shared_executor_->remove_node(node->get_node_base_interface());
            }
            lifecycle_robot_nodes_.erase(team_name);
        }

        // ✅ Remove executor lifecycle manager
        if (lifecycle_clients_.count(team_name) > 0) {
            auto lifecycle_client = lifecycle_clients_[team_name];
            if (!lifecycle_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE) ||
                !lifecycle_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN))
            {
                RCLCPP_WARN(this->get_logger(), "Failed to shutdown LifecycleServiceClient for team: %s", team_name.c_str());
            }
            shared_executor_->remove_node(lifecycle_client);
            lifecycle_clients_.erase(team_name);
        }

        // ✅ Remove executor node
        auto executor_node = team_nodes_[team_name];
        shared_executor_->remove_node(executor_node->get_node_base_interface());
        team_nodes_.erase(team_name);

        RCLCPP_INFO(this->get_logger(), "Team %s stopped. Simulators remain active.", team_name.c_str());
    }

    response->success = true;
    response->message = "Teams stopped successfully.";
}


void TeamLifecycleManager::stopAllExecutors() {
    RCLCPP_INFO(this->get_logger(), "Stopping all teams...");
    std::lock_guard<std::mutex> lock(thread_mutex_);

    // Stop lifecycle nodes
    for (auto &team_entry : lifecycle_robot_nodes_) {
        for (auto &node : team_entry.second) {
            auto current_state = node->get_current_state().id();

            if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                RCLCPP_INFO(this->get_logger(), "Deactivating node: %s", node->get_name());
                node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
            }

            current_state = node->get_current_state().id();
            if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                RCLCPP_INFO(this->get_logger(), "Shutting down node: %s", node->get_name());
                node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
            } else {
                RCLCPP_WARN(this->get_logger(), "Node %s is in unexpected state %u, skipping shutdown.", 
                            node->get_name(), current_state);
            }

            shared_executor_->remove_node(node->get_node_base_interface());
        }
    }

    lifecycle_robot_nodes_.clear();

    // Stop normal nodes
    for (auto &team_entry : normal_robot_nodes_) {
        for (auto &node : team_entry.second) {
            RCLCPP_INFO(this->get_logger(), "Removing normal node: %s", node->get_name());
            shared_executor_->remove_node(node);
        }
    }

    normal_robot_nodes_.clear();

    // Stop executor lifecycle managers
    for (auto &team_entry : lifecycle_clients_) {
        auto lifecycle_client = team_entry.second;
        RCLCPP_INFO(this->get_logger(), "Shutting down LifecycleServiceClient for team: %s", team_entry.first.c_str());
        if (!lifecycle_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
            RCLCPP_WARN(this->get_logger(), "Failed to deactivate LifecycleServiceClient for team: %s", team_entry.first.c_str());
        }
        if (!lifecycle_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN)) {
            RCLCPP_WARN(this->get_logger(), "Failed to shutdown LifecycleServiceClient for team: %s", team_entry.first.c_str());
        }
        shared_executor_->remove_node(lifecycle_client);
    }

    lifecycle_clients_.clear();

    // Stop executor nodes
    for (auto &team_entry : team_nodes_) {
        RCLCPP_INFO(this->get_logger(), "Removing executor node for team: %s", team_entry.first.c_str());
        shared_executor_->remove_node(team_entry.second->get_node_base_interface());
    }

    team_nodes_.clear();

    RCLCPP_INFO(this->get_logger(), "All teams stopped.");
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> TeamLifecycleManager::createActionNode(
    const std::string &action_type,
    const std::string &robot_name,
    const std::string &team_name)
{
    if (action_type == "change_site")
    {
        return std::make_shared<ChangeSiteActionNode>(robot_name, team_name);
    }
    else if (action_type == "landing")
    {
        return std::make_shared<LandingActionNode>(robot_name, team_name);
    }
    else if (action_type == "navigation_air")
    {
        return std::make_shared<NavigationAirActionNode>(robot_name, team_name);
    }
    else if (action_type == "navigation_water")
    {
        return std::make_shared<NavigationWaterActionNode>(robot_name, team_name);
    }
    else if (action_type == "observe_2r")
    {
        return std::make_shared<Observe2rActionNode>(robot_name, team_name);
    }
    else if (action_type == "observe")
    {
        return std::make_shared<ObserveActionNode>(robot_name, team_name);
    }
    else if (action_type == "sample")
    {
        return std::make_shared<SampleActionNode>(robot_name, team_name);
    }
    else if (action_type == "switch_airwater")
    {
        return std::make_shared<SwitchAirWaterActionNode>(robot_name, team_name);
    }
    else if (action_type == "switch_waterair")
    {
        return std::make_shared<SwitchWaterAirActionNode>(robot_name, team_name);
    }
    else if (action_type == "takeoff")
    {
        return std::make_shared<TakeoffActionNode>(robot_name, team_name);
    }
    else if (action_type == "translate_data")
    {
        return std::make_shared<TranslateDataActionNode>(robot_name, team_name);
    }
    else
    {
        throw std::invalid_argument("Unsupported action type: " + action_type);
    }
}


std::vector<std::string> TeamLifecycleManager::getAllActionTypes()
{
    return {
        "change_site",
        "landing",
        "navigation_air",
        "navigation_water",
        "observe_2r",
        "observe",
        "sample",
        "switch_airwater",
        "switch_waterair",
        "takeoff",
        "translate_data"};
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeamLifecycleManager>();

    std::thread spin_thread([&]() {
        rclcpp::spin(node);
    });

    RCLCPP_INFO(node->get_logger(), "Team Lifecycle Manager is running.");
    spin_thread.join();

    rclcpp::shutdown();
    return 0;
}
