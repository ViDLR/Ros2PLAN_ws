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

void ExecutionManagerNode::executionStatusCallback(
    const std_msgs::msg::String::SharedPtr msg)
{
    const std::string& status = msg->data;

    if (status.rfind("FAILURE", 0) == 0) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Detected FAILURE status: '%s'", status.c_str());

        // 🔍 Step 1: Extract failure info
        std::map<std::string, std::string> failure_data = parseFailureStatus(status);

        // 🔽 Step 2: Write failure_info.json
        {
            nlohmann::json failure_json;
            failure_json["robot"] = failure_data["robot"];
            failure_json["action"] = failure_data["action"];
            failure_json["failure_index"] = failure_data["failure_index"];
            failure_json["site"] = failure_data["site"];
            failure_json["point"] = failure_data["point"];

            std::ofstream f("/tmp/failure_info.json");
            if (!f) {
                RCLCPP_ERROR(this->get_logger(), "❌ Could not write /tmp/failure_info.json");
                return;
            }
            f << failure_json.dump(4);
        }

        // 🐍 Step 3: Call Python script to analyze the failure and its impact on plans
        std::string cmd = "python3 " + std::string(ament_index_cpp::get_package_share_directory("action_simulator")) + "/scripts/handle_failure.py";
        int result = std::system(cmd.c_str());

        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to run failure_problem_creator.py");
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "✅ Successfully updated the mission with failure");
        }


        // 🧪 Step 5: Validate impact and replan if needed
        validateFailureImpact(failure_data["team"]);
    }
}


std::map<std::string, std::string> ExecutionManagerNode::parseFailureStatus(const std::string& msg)
{
    std::map<std::string, std::string> result;

    std::regex pattern(R"(team:\s*(\S+)\s*\|\s*robot:\s*(\S+)\s*\|\s*action:\s*(\S+)\s*\|\s*point:\s*(\S+)\s*\|\s*site:\s*(\S+)\s*\|\s*index:\s*(-?\d+))");
    std::smatch matches;

    if (std::regex_search(msg, matches, pattern) && matches.size() == 7) {
        result["team"] = matches[1].str();
        result["robot"] = matches[2].str();
        result["action"] = matches[3].str();
        result["point"] = matches[4].str();
        result["site"] = matches[5].str();
        result["index"] = matches[6].str();
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to parse failure status string: [%s]", msg.c_str());
    }

    return result;
}


void ExecutionManagerNode::applyFailureToProblem(const std::map<std::string, std::string>& failure_data)
{
    using json = nlohmann::json;

    std::string robot = failure_data.at("robot");
    std::string action = failure_data.at("action");
    std::string point  = failure_data.at("point");
    std::string site   = failure_data.at("site");
    int index          = std::stoi(failure_data.at("index"));

    // 🔁 Load failure config JSON
    std::ifstream file("/tmp/failure_index.json");
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open failure_index.json");
        return;
    }

    json failure_config;
    file >> failure_config;

    if (!failure_config.contains(action)) {
        RCLCPP_ERROR(this->get_logger(), "Failure config missing entry for action [%s]", action.c_str());
        return;
    }

    auto failures = failure_config[action];
    if (index < 0 || index >= static_cast<int>(failures.size())) {
        RCLCPP_ERROR(this->get_logger(), "Invalid failure index %d for action %s", index, action.c_str());
        return;
    }

    std::string failure_type = failures[std::to_string(index)];

    RCLCPP_WARN(this->get_logger(), "⚠️ Applying failure type: [%s]", failure_type.c_str());

    // 🔁 Load original problem.pddl
    std::ifstream problem_in("/tmp/problem.pddl");
    if (!problem_in.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open original problem.pddl");
        return;
    }

    std::ofstream problem_out("/tmp/validation/failed_problem.pddl");
    if (!problem_out.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not create failed_problem.pddl");
        return;
    }

    std::string line;
    while (std::getline(problem_in, line)) {
        // --- CASE: robot_unavailable
        if (failure_type == "robot_unavailable") {
            if (line.find("(available " + robot + ")") != std::string::npos) continue;
            if (line.find("(cansample " + robot + ")") != std::string::npos) continue;
            if (line.find("(canrelay " + robot + ")") != std::string::npos) continue;
            if (line.find("(canswitch " + robot + ")") != std::string::npos) continue;
            if (line.find("(cantranslate " + robot + ")") != std::string::npos) continue;
        }

        // --- CASE: point_unswitchable
        if (failure_type == "point_unswitchable") {
            if (line.find("(isswitchable " + point + ")") != std::string::npos) continue;
        }

        // --- CASE: robot_cannot_translate
        if (failure_type == "robot_cannot_translate") {
            if (line.find("(cantranslate " + robot + ")") != std::string::npos) continue;
        }

        // --- CASE: robot_cannot_sample
        if (failure_type == "robot_cannot_sample") {
            if (line.find("(cansample " + robot + ")") != std::string::npos) continue;
        }

        // --- CASE: robot_cannot_switch X (e.g., robot_waterconf or robot_airconf)
        if (failure_type.find("robot_cannot_switch") != std::string::npos) {
            if (line.find("(canswitch " + robot + ")") != std::string::npos) continue;
        }

        // TODO: Add further specific cases like "remove_goal_point", "create_new_poi" etc.

        problem_out << line << "\n";
    }

    problem_in.close();
    problem_out.close();

    RCLCPP_INFO(this->get_logger(), "🛠️ Applied failure [%s], saved to failed_problem.pddl", failure_type.c_str());
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
    const std::map<std::string, plansys2_msgs::msg::Plan> &labeled_plans)
{
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
            std::string valid_team_name = "team_" + path_name;

            // Extract sites
            std::string sites_str = line.substr(colon_pos + 2, robots_pos - colon_pos - 2);
            sites = splitString(sites_str, ',');

            // Extract and sanitize robots
            std::string robots_str = line.substr(robots_pos + 8, dependencies_pos - robots_pos - 8);
            robots = splitString(robots_str, ',');

            for (auto &robot : robots) {
                robot.erase(std::remove_if(robot.begin(), robot.end(), [](char c) {
                    return c == '[' || c == ']' || c == '\'' || c == ' ';
                }), robot.end());
            }

            // Extract and sanitize dependencies
            std::vector<std::string> formatted_dependencies;
            if (dependencies_pos != std::string::npos) {
                std::string dependencies_str = line.substr(dependencies_pos + 15);
                dependencies = splitString(dependencies_str, ',');

                for (auto &dep : dependencies) {
                    dep.erase(std::remove_if(dep.begin(), dep.end(), [](char c) {
                        return c == '[' || c == ']' || c == '\'' || c == ' ';
                    }), dep.end());

                    if (!dep.empty()) {
                        formatted_dependencies.push_back("team_" + dep);
                    }
                }
            }

            site_paths[valid_team_name] = sites;
            robot_paths[valid_team_name] = robots;
            team_dependencies_[valid_team_name] = formatted_dependencies;

            plansys2_msgs::msg::Team team;
            team.name = valid_team_name;
            team.robots = robots;
            teams.push_back(team);

            // ✅ Assign plan to team using path_name as label
            if (labeled_plans.find(path_name) != labeled_plans.end()) {
                teams_plans[valid_team_name] = labeled_plans.at(path_name);
            } else {
                RCLCPP_WARN(this->get_logger(), "⚠️ No plan found for path label: %s", path_name.c_str());
            }
        }
    }

    file.close();

    // ✅ Store team dependencies correctly
    this->team_dependencies_ = team_dependencies_;

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
    std::string package_prefix = ament_index_cpp::get_package_prefix("action_simulator");
    std::filesystem::path validator_path = std::filesystem::path(package_prefix) / "lib/action_simulator/external_validator/val-pddl";
    
    std::string output_file = "/tmp/validation/val_output_" + team_name + ".txt";
    std::string validator_cmd = validator_path.string() + 
        " -c -t 0.001 domain-optic.pddl /tmp/validation/failed_problem.pddl /tmp/plan_output/subproblems/" + team_name + ".pddl > " + output_file;

    int result = std::system(validator_cmd.c_str());

    if (result == 0) {
        RCLCPP_WARN(this->get_logger(), "Failure in '%s' does not affect other paths.", team_name.c_str());
        stn_controller_->handleFailure(team_name);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failure in '%s' requires global replanning.", team_name.c_str());

        // std::vector<std::string> affected_paths = {team_name};
        // std::string replan_mode = "incremental";

        // auto domain = domain_client_->getDomain();
        // auto problem = problem_client_->getProblem();

        // // 🔄 Get labeled plans
        // auto labeled_plans = planner_client_->getMultiPathPlan(
        //     domain, problem, "", 300s, replan_mode, affected_paths);

        // if (labeled_plans.empty()) {
        //     RCLCPP_ERROR(this->get_logger(), "Replanning failed: no plans returned.");
        //     return;
        // }

        // // 🔁 Parse ARMS output with labeled plans
        // parseArmsResult("/tmp/plan_output/arms_result.txt", labeled_plans);

        // RCLCPP_INFO(this->get_logger(), "Replanning complete. Restarting execution.");

        // // 🔁 Reinitialize STN
        // stn_controller_->initializePaths(active_teams, teams_plans, team_dependencies_);
    }
}


ExecutionManagerNode::ValResult ExecutionManagerNode::parse_val_output(const std::string &file_path) {
    ValResult result;
    result.status = "UNKNOWN";
    result.value = -1;

    std::ifstream infile(file_path);
    std::string line;
    while (std::getline(infile, line)) {
        if (line.find("Plan valid") != std::string::npos) {
            result.status = "SUCCESS";
        } else if (line.find("Plan failed") != std::string::npos) {
            result.status = "FAILURE";
        } else if (line.find("Invariant for") != std::string::npos) {
            auto pos = line.find("Invariant for");
            result.failed_actions.push_back(line.substr(pos + 13));
        } else if (line.find("Set (") != std::string::npos || line.find("Set (at") != std::string::npos) {
            result.repair_advice.push_back(line);
        } else if (line.find("Final value:") != std::string::npos) {
            result.value = std::stoi(line.substr(line.find(":") + 1));
        }
    }

    return result;
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

void ExecutionManagerNode::load_and_save_failure_index(const std::string &src_path) {
    RCLCPP_INFO(this->get_logger(), "Loading failure index from: %s", src_path.c_str());

    std::ifstream file(src_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open failure index JSON: %s", src_path.c_str());
        return;
    }

    nlohmann::json failure_data;
    try {
        file >> failure_data;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error parsing failure index JSON: %s", e.what());
        return;
    }
    file.close();

    const std::string tmp_file_path = "/tmp/failure_index.json";
    std::ofstream tmp_file(tmp_file_path);
    if (!tmp_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write failure index to: %s", tmp_file_path.c_str());
        return;
    }

    try {
        tmp_file << failure_data.dump(4);
        RCLCPP_INFO(this->get_logger(), "Failure index successfully saved to: %s", tmp_file_path.c_str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error writing failure index JSON: %s", e.what());
    }

    tmp_file.close();
}

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
    
    load_and_save_failure_index("src/my_examples/plansys2_testexample/pddl/failure_index.json");

    // Fetch and analyze the plan
    auto domain = domain_client_->getDomain();
    auto problem = problem_client_->getProblem();

    // ✅ Use the updated planner client call (returns map<label, plan>)
    std::map<std::string, plansys2_msgs::msg::Plan> labeled_plans =
        planner_client_->getMultiPathPlan(domain, problem, "", 300s, "");

    if (labeled_plans.empty()) {
    RCLCPP_ERROR(this->get_logger(), "❌ Failed to fetch multi-path plans.");
    return;
    }

    RCLCPP_INFO(this->get_logger(), "🔍 Received %zu labeled plans from Multi-Path Planner", labeled_plans.size());

    // ✅ Print raw plans before assignment
    for (const auto& [label, plan] : labeled_plans) {
    RCLCPP_INFO(this->get_logger(), "📜 Plan [%s] -> Steps: %zu", label.c_str(), plan.items.size());
    for (const auto& item : plan.items) {
        RCLCPP_INFO(this->get_logger(), "  - [%f] (%s)", item.time, item.action.c_str());
    }
    }

    RCLCPP_INFO(this->get_logger(), "📦 Total labeled plans fetched: %lu", labeled_plans.size());

    // Parse ARMS result and distribute plans
    // (You likely use plan labels here for assignment or merging)
    parseArmsResult("/tmp/plan_output/arms_result.txt", labeled_plans);

    // Check that team plans are properly assigned
    RCLCPP_INFO(this->get_logger(), "Checking active teams before sending to STN: %zu teams.", active_teams.size());

    // ✅ Initialize STN with parsed data
    stn_controller_->initializePaths(active_teams, teams_plans, team_dependencies_);

    // ✅ Use MultiThreadedExecutor in a separate thread
    executor_->add_node(stn_controller_);   // STN Node
    spin_thread_ = std::thread([this]() {
    executor_->spin();
    });

    

    std::this_thread::sleep_for(std::chrono::seconds(5)); // Small delay to ensure nodes are active

    publish_world_info("src/my_examples/plansys2_testexample/pddl/world_info.json");    
    stn_controller_->triggerInitialExecutions();

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
