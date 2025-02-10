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


void ExecutionManagerNode::parseArmsResult(const std::string &file_path, 
                                           const std::vector<plansys2_msgs::msg::Plan> &plans) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open ARMS result file: %s", file_path.c_str());
        return;
    }

    std::string line;
    std::vector<std::string> team_names;
    std::map<std::string, std::vector<std::string>> robot_paths;
    std::map<std::string, std::vector<std::string>> site_paths;
    std::map<std::string, std::vector<std::string>> dependencies;
    
    while (std::getline(file, line)) {
        if (line.rfind("path", 0) == 0) {  // Detect path lines
            std::stringstream ss(line);
            std::string label;
            int path_index;
            ss >> label >> path_index;  // Read "path X"

            std::vector<std::string> sites;
            std::string site;
            while (ss >> site && site != "robots:") {
                sites.push_back(site);
            }

            std::vector<std::string> robots;
            std::string robot;
            while (ss >> robot) {
                robots.push_back(robot);
            }
            
            std::string team_name = "team" + std::to_string(path_index);
            team_names.push_back(team_name);
            site_paths[team_name] = sites;
            robot_paths[team_name] = robots;
        } else if (line.rfind("depends", 0) == 0) {  // Detect dependency lines
            std::stringstream ss(line);
            std::string label, dependent, dependency;
            ss >> label >> dependent >> dependency;  // Read "depends pathX pathY"
            dependencies[dependent].push_back(dependency);
        }
    }
    
    file.close();

    // Assign dependencies
    for (const auto &entry : dependencies) {
        stn_controller_->path_dependencies_[entry.first] = entry.second;
    }

    // Logging parsed paths and creating teams
    for (size_t i = 0; i < team_names.size() && i < plans.size(); ++i) {
        const std::string &team_name = team_names[i];
        const std::vector<std::string> &robots = robot_paths[team_name];
        const std::vector<std::string> &sites = site_paths[team_name];

        plansys2_msgs::msg::Team team;
        team.name = team_name;
        team.robots = robots;
        active_teams.push_back(team);

        teams_plans[team_name] = plans[i];

        RCLCPP_INFO(this->get_logger(), "Created Team: %s | Robots: [%s] | Sites: [%s] | Plan assigned",
                    team_name.c_str(),
                    fmt::format("{}", fmt::join(robots, ", ")).c_str(),
                    fmt::format("{}", fmt::join(sites, ", ")).c_str());
    }
}




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

void ExecutionManagerNode::addExecutorCallbacks(const std::vector<plansys2_msgs::msg::Team> &active_teams) {
    
    for (const auto &team : active_teams) {
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

    // Corrected: Store the created ExecutorClient in the map
    executor_clients_[team_name] = std::make_shared<plansys2::ExecutorClient>(
        "executor_client_" + team_name,  // Node name
        team_name,                        // Namespace
        "executor_" + team_name           // Executor name
    );

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
            createExecutorClient(team.name);  // Now correctly storing in `executor_clients_`
        } else {
            RCLCPP_INFO(this->get_logger(), "Executor client for team '%s' already exists. No action taken.", team.name.c_str());
        }
    }
}

void ExecutionManagerNode::executionFeedbackCallback(const std::string &team_name, float progress) {
    stn_controller_->trackExecutionProgress(team_name, "placeholdersite",progress);
    
    if (progress >= 1.0) {
        RCLCPP_INFO(this->get_logger(), "Team '%s' completed execution.", team_name.c_str());
    }

    startPlanExecution();
}

void ExecutionManagerNode::handleFailure(const std::string& team_name) {
    RCLCPP_WARN(this->get_logger(), "Handling failure for team '%s'.", team_name.c_str());

    validateFailureImpact(team_name);
    stn_controller_->handleFailure(team_name);
}

void ExecutionManagerNode::propagateDelay(const std::string& team_name, float delay) {
    RCLCPP_WARN(this->get_logger(), "Propagating delay of %f seconds in team '%s'.", delay, team_name.c_str());

    stn_controller_->propagateDelay(team_name, delay);

    for (const auto &dep : stn_controller_->path_dependencies_[team_name]) {
        RCLCPP_WARN(this->get_logger(), "Delaying dependent path '%s' due to delay in '%s'.", dep.c_str(), team_name.c_str());
    }
}


void ExecutionManagerNode::validateFailureImpact(const std::string& team_name) {
    std::string validator_cmd = "./val-exec /tmp/validator/subproblems/" + team_name + ".pddl";
    int result = std::system(validator_cmd.c_str());

    if (result == 0) {
        RCLCPP_WARN(this->get_logger(), "Failure in '%s' does not affect other paths. Adapting locally.", team_name.c_str());
        startPlanExecution();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failure in '%s' requires global replanning.", team_name.c_str());
        
        std::vector<std::string> affected_paths = {team_name};
        std::string replan_mode = "incremental";
        
        auto domain = domain_client_->getDomain();
        auto problem = problem_client_->getProblem();
        auto plans = planner_client_->getMultiPathPlan(domain, problem, "", 15s, replan_mode, affected_paths);
        
        parseArmsResult("/tmp/plan_output/arms_result.txt", plans);
        startPlanExecution();
    }
}


void ExecutionManagerNode::removeExecutorClients(const std::vector<std::string> &team_names) {
    for (const auto &team_name : team_names) {
        removeExecutorClient(team_name);
    }
}

// void ExecutionManagerNode::initializeSTN() {
//     stn_controller = st
// }


void ExecutionManagerNode::startPlanExecution() {
    for (const auto &team : active_teams) {
        if (!executor_clients_.count(team.name)) {
            RCLCPP_WARN(this->get_logger(), "No executor client for '%s'.", team.name.c_str());
            continue;
        }

        if (!stn_controller_->isPathReady(team.name)) {
            RCLCPP_WARN(this->get_logger(), "Team '%s' is waiting on dependencies.", team.name.c_str());
            continue;
        }

        auto client = executor_clients_[team.name];
        RCLCPP_INFO(this->get_logger(), "Starting execution for team '%s'.", team.name.c_str());

        auto team_plan = teams_plans[team.name];

        if (!client->start_plan_execution(team_plan)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start execution for '%s'.", team.name.c_str());
        } else {
            stn_controller_->trackExecutionProgress(team.name, "placeholdersite", 0.1);
        }
    }
}


void ExecutionManagerNode::ExecutionSequenceFunction()
{
    RCLCPP_INFO(this->get_logger(), "Starting ExecutionSequenceFunction ...");

    // Initialize clients
    domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();

    // Load the problem from a .pddl file
    std::ifstream problem_file("src/my_examples/plansys2_testexample/pddl/problem_hightest.pddl");
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
        domain, problem, "", 15s, replan_mode, affected_paths);

    if (plans.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to fetch multi-path plans.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Fetched %lu path-specific plans.", plans.size());

    // Parse ARMS result and distribute plans
    parseArmsResult("/tmp/plan_output/arms_result.txt", plans);

    // Intialize the STN temporal controller
    stn_controller_ = std::make_shared<STNController>();
    stn_controller_->initializePaths(active_teams, teams_plans);

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

    // Call the /start_teams service of TLCMN for starting teams
    RCLCPP_INFO(this->get_logger(), "Calling the team creation client...");
    auto start_teams_client = this->create_client<StartTeams>("/start_teams");

    if (!start_teams_client->wait_for_service(20s)) {
        RCLCPP_ERROR(this->get_logger(), "/start_teams service not available");
        return;
    }

    auto request = std::make_shared<StartTeams::Request>();
    request->teams = active_teams;

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

    // Create callbacks and executor clients for the teams if they don’t exist
    addExecutorCallbacks(active_teams);
    addExecutorClients(active_teams);

    // Feed the teams with the world information (problem information)
    publish_world_info("src/my_examples/plansys2_testexample/pddl/world_info.json");

}




}  // namespace action_simulator
