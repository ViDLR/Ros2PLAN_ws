#include "action_simulator/ExecutionManagerNode.hpp"

namespace action_simulator {

using namespace std::chrono_literals;
using StartTeams = plansys2_msgs::srv::StartTeams;

ExecutionManagerNode::ExecutionManagerNode() : rclcpp::Node("execution_manager_node")
{
    RCLCPP_INFO(this->get_logger(), "Creating ExecutionManagerNode...");
    world_info_publisher_ = this->create_publisher<plansys2_msgs::msg::WorldInfo>("/world_info", 10);
    execution_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/stn_execution_status", 10,
        std::bind(&ExecutionManagerNode::executionStatusCallback, this, std::placeholders::_1));
    ExecutionSequenceFunction();
}

ExecutionManagerNode::~ExecutionManagerNode() {
    if (executor_) {
        RCLCPP_INFO(this->get_logger(), "Shutting down MultiThreadedExecutor...");
        executor_->cancel();
    }

    if (spin_thread_.joinable()) {
        RCLCPP_INFO(this->get_logger(), "Joining spin thread...");
        spin_thread_.join();
    }
}

void ExecutionManagerNode::executionStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data.find("fail") != std::string::npos) {
        handleFailure(msg->data);
    }
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



void ExecutionManagerNode::load_and_save_world_info(const std::string &problem_info_path) 
{
    RCLCPP_INFO(this->get_logger(), "Loading WorldInfo from JSON file: %s", problem_info_path.c_str());

    // Read JSON file
    std::ifstream file(problem_info_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open world_info.json file: %s", problem_info_path.c_str());
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

    // Save the JSON data to /tmp/worldinfo.json
    const std::string tmp_file_path = "/tmp/world_info.json";
    std::ofstream tmp_file(tmp_file_path);
    if (!tmp_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write JSON file to: %s", tmp_file_path.c_str());
        return;
    }

    try {
        tmp_file << world_data.dump(4);  // Pretty print with 4-space indentation
        RCLCPP_INFO(this->get_logger(), "WorldInfo JSON successfully saved to: %s", tmp_file_path.c_str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error writing to JSON file: %s", e.what());
        return;
    }
    tmp_file.close();

}

void ExecutionManagerNode::parseArmsResult(
    const std::string &file_path,
    const std::vector<plansys2_msgs::msg::Plan> &plans) {
    
    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open ARMS result file: %s", file_path.c_str());
        return;
    }

    std::string line;
    
    std::map<std::string, std::vector<std::string>> team_dependencies_;
    std::map<std::string, std::vector<std::string>> robot_paths;
    std::map<std::string, std::vector<std::string>> site_paths;
    
    std::vector<plansys2_msgs::msg::Team> teams;
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string label;
        ss >> label;

        if (label.rfind("path", 0) == 0) {  // Line starts with "path"
            std::string path_name;
            std::vector<std::string> sites, robots, dependencies;

            size_t colon_pos = line.find(":");
            size_t robots_pos = line.find("robots:");
            size_t dependencies_pos = line.find("| Predecessors:");

            if (colon_pos == std::string::npos || robots_pos == std::string::npos) continue;

            path_name = line.substr(5, colon_pos - 5); // Extract path name
            std::string valid_team_name = "team_" + path_name;  // ✅ Prefix with "team_"

            // Extract sites
            std::string sites_str = line.substr(colon_pos + 2, robots_pos - colon_pos - 2);
            sites = splitString(sites_str, ',');

            // Extract and **sanitize** robots
            std::string robots_str = line.substr(robots_pos + 8, dependencies_pos - robots_pos - 8);
            robots = splitString(robots_str, ',');

            for (auto &robot : robots) {
                robot.erase(std::remove_if(robot.begin(), robot.end(), [](char c) {
                    return c == '[' || c == ']' || c == '\'' || c == ' ';
                }), robot.end());  // ✅ Trim unwanted characters
            }

            // Extract dependencies and convert to valid team names
            std::vector<std::string> formatted_dependencies;
            if (dependencies_pos != std::string::npos) {
                std::string dependencies_str = line.substr(dependencies_pos + 15);
                dependencies = splitString(dependencies_str, ',');

                for (auto &dep : dependencies) {
                    dep.erase(std::remove_if(dep.begin(), dep.end(), [](char c) {
                        return c == '[' || c == ']' || c == '\'' || c == ' ';
                    }), dep.end());  // ✅ Trim unwanted characters

                    if (!dep.empty()) {
                        formatted_dependencies.push_back("team_" + dep);
                    }
                }
            }

            site_paths[valid_team_name] = sites;
            robot_paths[valid_team_name] = robots;
            team_dependencies_[valid_team_name] = formatted_dependencies;

            // ✅ Store cleaned team information
            plansys2_msgs::msg::Team team;
            team.name = valid_team_name;
            team.robots = robots;
            teams.push_back(team);
        }
    }

    file.close();

    // ✅ Store team dependencies correctly
    this->team_dependencies_ = team_dependencies_;

    // ✅ Assign plans to corresponding teams
    if (plans.size() != teams.size()) {
        RCLCPP_ERROR(this->get_logger(), "Mismatch between parsed teams and provided plans.");
        return;
    }

    for (size_t i = 0; i < teams.size(); i++) {
        teams_plans[teams[i].name] = plans[i];
    }

    // ✅ Finally, update `active_teams`
    active_teams = teams;

    // ✅ Debugging log
    RCLCPP_INFO(this->get_logger(), "Parsed %zu teams successfully.", active_teams.size());

    for (const auto& team : active_teams) {
        RCLCPP_INFO(this->get_logger(), "Active Team: %s | Robots: %s | Dependencies: %s", 
                    team.name.c_str(),
                    fmt::format("{}", fmt::join(team.robots, ", ")).c_str(),
                    fmt::format("{}", fmt::join(team_dependencies_[team.name], ", ")).c_str());
    }
}





// ✅ **Fix: Add `splitString` Implementation**
std::vector<std::string> ExecutionManagerNode::splitString(const std::string &input, char delimiter) {
    std::vector<std::string> result;
    std::stringstream ss(input);
    std::string item;
    
    while (std::getline(ss, item, delimiter)) {
        item.erase(std::remove(item.begin(), item.end(), '\''), item.end()); // Remove single quotes
        item.erase(std::remove(item.begin(), item.end(), ' '), item.end());  // Trim spaces
        if (!item.empty()) {
            result.push_back(item);
        }
    }
    return result;
}

void ExecutionManagerNode::handleFailure(const std::string& team_name) {
    RCLCPP_WARN(this->get_logger(), "Handling failure for team '%s'.", team_name.c_str());

    validateFailureImpact(team_name);
    stn_controller_->handleFailure(team_name);

    // ✅ Restart execution if failure is resolved
    if (stn_controller_->isPathReady(team_name)) {
        stn_controller_->startTeamExecution(team_name);
    }
}

void ExecutionManagerNode::validateFailureImpact(const std::string& team_name) {
    std::string validator_cmd = "./val-exec /tmp/validator/subproblems/" + team_name + ".pddl";
    int result = std::system(validator_cmd.c_str());

    if (result == 0) {
        RCLCPP_WARN(this->get_logger(), "Failure in '%s' does not affect other paths.", team_name.c_str());
        stn_controller_->handleFailure(team_name);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failure in '%s' requires global replanning.", team_name.c_str());
        
        std::vector<std::string> affected_paths = {team_name};
        std::string replan_mode = "incremental";
        
        auto domain = domain_client_->getDomain();
        auto problem = problem_client_->getProblem();
        auto plans = planner_client_->getMultiPathPlan(domain, problem, "", 300s, replan_mode, affected_paths);
        
        parseArmsResult("/tmp/plan_output/arms_result.txt", plans);

        RCLCPP_INFO(this->get_logger(), "Replanning complete. Restarting execution.");
        stn_controller_->initializePaths(active_teams, teams_plans, team_dependencies_);
    }
}

// void ExecutionManagerNode::startPlanExecution() {
//     for (const auto &team : active_teams) {
//         if (!executor_clients_.count(team.name)) {
//             RCLCPP_WARN(this->get_logger(), "No executor client for '%s'.", team.name.c_str());
//             continue;
//         }

//         if (!stn_controller_->isPathReady(team.name)) {
//             RCLCPP_WARN(this->get_logger(), "Team '%s' is waiting on dependencies.", team.name.c_str());
//             continue;
//         }

//         auto client = executor_clients_[team.name];
//         RCLCPP_INFO(this->get_logger(), "Starting execution for team '%s'.", team.name.c_str());

//         auto team_plan = teams_plans[team.name];

//         if (!client->start_plan_execution(team_plan)) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to start execution for '%s'.", team.name.c_str());
//         } else {
//             stn_controller_->trackExecutionProgress(team.name, 0.1);
//         }
//     }
// }


void ExecutionManagerNode::ExecutionSequenceFunction()
{
    RCLCPP_INFO(this->get_logger(), "Starting ExecutionSequenceFunction ...");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    // ✅ Initialize STNController
    stn_controller_ = std::make_shared<STNController>();

    // Initialize clients
    domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();

    // Load the problem from a .pddl file
    std::ifstream problem_file("src/my_examples/plansys2_testexample/pddl/problem_7r7s_iros.pddl");
    if (!problem_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open problem file.");
        throw std::runtime_error("Problem file load failed");
    }
    std::string problem_str((std::istreambuf_iterator<char>(problem_file)),
                            std::istreambuf_iterator<char>());
    problem_file.close();

    problem_client_->addProblem(problem_str);

    // Load the problem state from a JSON file
    load_and_save_world_info("src/my_examples/plansys2_testexample/pddl/world_info.json");

    RCLCPP_INFO(this->get_logger(), "Activating ExecutionManagerNode...");

    // Multi-path plan request
    std::string replan_mode = "full";  // Can be 'full' or 'incremental'
    std::vector<std::string> affected_paths = {"path1", "path2", "path3", "path4", "path5"};

    // Fetch and analyze the plan
    auto domain = domain_client_->getDomain();
    auto problem = problem_client_->getProblem();
    // Fetch multi-path plans from the planner
    std::vector<plansys2_msgs::msg::Plan> plans = planner_client_->getMultiPathPlan(
        domain, problem, "", 300s, replan_mode, affected_paths);

    if (plans.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to fetch multi-path plans.");
        return;
    }


    RCLCPP_INFO(this->get_logger(), "🔍 Received %zu plans from Multi-Path Planner", plans.size());

    // ✅ Print raw plans before assignment
    int i = 0;
    for (const auto& plan : plans) {
        RCLCPP_INFO(this->get_logger(), "📜 Plan [%d] -> Steps: %zu", i, plan.items.size());
        for (const auto& item : plan.items) {
            RCLCPP_INFO(this->get_logger(), "  - [%f] (%s)", item.time, item.action.c_str());
        }
        i++;
    }

    RCLCPP_INFO(this->get_logger(), "Fetched %lu path-specific plans.", plans.size());

    // Parse ARMS result and distribute plans
    parseArmsResult("/tmp/plan_output/arms_result.txt", plans);
    // Ensure `active_teams` is filled properly
    RCLCPP_INFO(this->get_logger(), "Checking active teams before sending to STN: %zu teams.", active_teams.size());
    
    // // ✅ Print parsed plans after ARMS processing
    // RCLCPP_INFO(this->get_logger(), "📜 Parsed Plans in teams_plans:");
    // for (const auto& [team_name, plan] : teams_plans) {  // Correct way to iterate over a map
    //     RCLCPP_INFO(this->get_logger(), "📜 Team [%s] -> Steps: %zu", team_name.c_str(), plan.items.size());
    //     for (const auto& item : plan.items) {
    //         RCLCPP_INFO(this->get_logger(), "  - [%f] (%s)", item.time, item.action.c_str());
    //     }
    // }
    // ✅ Initialize STN with parsed data
    stn_controller_->initializePaths(active_teams, teams_plans, team_dependencies_);
    // ✅ Use MultiThreadedExecutor in a separate thread
    executor_->add_node(stn_controller_);   // STN Node
    // ✅ Launch spin in a separate thread
    spin_thread_ = std::thread([this]() {
        executor_->spin();
    });

    std::this_thread::sleep_for(std::chrono::seconds(5)); // Small delay to ensure nodes are active
    publish_world_info("src/my_examples/plansys2_testexample/pddl/world_info.json");
    stn_controller_->triggerInitialExecutions();

    // ExecutionSequenceFunction continues running other logic
    RCLCPP_INFO(this->get_logger(), "ExecutionManagerNode is running ESF logic after spinning...");
    // // Notify that STN is in control
    // RCLCPP_INFO(this->get_logger(), "STN Initialized. Monitoring execution...");

    // // STN now handles team execution and monitoring
    // stn_controller_->startMonitoring();

    // // Start execution of independent teams
    // std::vector<std::string> initial_teams = stn_controller_->getInitialExecutableTeams();
    // for (const auto &team_name : initial_teams) {
    //     startTeamExecution(team_name);
    // }

    // Simulated example teams matching the paths
    // std::vector<plansys2_msgs::msg::Team> teams;
    // std::map<std::string, plansys2_msgs::msg::Plan> teams_plans; // Map team to its corresponding plan

    // std::vector<std::string> team_names = {"team1", "team2", "team3", "team4", "team5"};
    // for (size_t i = 0; i < team_names.size() && i < plans.size(); ++i) {
    //     plansys2_msgs::msg::Team team;
    //     team.name = team_names[i];
    //     team.robots = {"robot" + std::to_string(i * 2), "robot" + std::to_string(i * 2 + 1)};
    //     teams.push_back(team);
    //     teams_plans[team.name] = plans[i]; // Assign the correct plan to the corresponding team
    // }

    // // Call the /start_teams service of TLCMN for starting teams
    // RCLCPP_INFO(this->get_logger(), "Calling the team creation client...");
    // auto start_teams_client = this->create_client<StartTeams>("/start_teams");

    // if (!start_teams_client->wait_for_service(20s)) {
    //     RCLCPP_ERROR(this->get_logger(), "/start_teams service not available");
    //     return;
    // }

    // auto request = std::make_shared<StartTeams::Request>();
    // request->teams = active_teams;

    // auto future = start_teams_client->async_send_request(request);

    // // Spin until the future is complete
    // auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

    // if (result != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(this->get_logger(), "Service call to /start_teams failed or timed out");
    //     return;
    // }

    // RCLCPP_INFO(this->get_logger(), "Service call completed. Fetching response...");
    // auto response = future.get();
    // if (!response->success) {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to start teams: %s", response->message.c_str());
    //     return;
    // }

    // RCLCPP_INFO(this->get_logger(), "Teams successfully started: %s", response->message.c_str());

    // // Create callbacks and executor clients for the teams if they don’t exist
    // addExecutorCallbacks(active_teams);
    // addExecutorClients(active_teams);

    // Feed the teams with the world information (problem information)
    // publish_world_info("src/my_examples/plansys2_testexample/pddl/world_info.json");

}




}  // namespace action_simulator
