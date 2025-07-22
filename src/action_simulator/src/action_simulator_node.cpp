#include "action_simulator/action_simulator_node.hpp"
#include <sstream>

using namespace std::literals::chrono_literals;
SimulationNode::SimulationNode(const std::string &robot_name, const std::string &team_name)
: Node("simulation_node_" + robot_name, team_name) // Pass namespace to the Node constructor
{
    // Fetch parameters and populate robot state
    this->declare_parameter<std::string>("robot_name", robot_name);
    this->declare_parameter<std::string>("team_name", team_name);

    this->get_parameter("robot_name", robot_state_.robot_name);
    this->get_parameter("team_name", robot_state_.team);

    // Populate default robot capabilities
    robot_state_.capabilities = {"canrelay", "canswitch", "cansample"};


    // Set up topics based on robot name
    std::string info_topic = "/simulation_info_" + robot_state_.robot_name;
    std::string result_topic = "/simulation_result_" + robot_state_.robot_name;
    std::string failure_topic = "/failing_actions_" + robot_state_.robot_name;
    std::string plan_topic = "/plan_" + robot_state_.robot_name;


    // Subscriptions
    action_subscription_ = this->create_subscription<plansys2_msgs::msg::ActionExecutionInfo>(
        info_topic, 10, std::bind(&SimulationNode::action_callback, this, std::placeholders::_1));
    failure_subscription_ = this->create_subscription<plansys2_msgs::msg::Failure>(
        failure_topic, 10, std::bind(&SimulationNode::failure_callback, this, std::placeholders::_1));
    world_info_subscription_ = this->create_subscription<plansys2_msgs::msg::WorldInfo>(
        "/world_info", 10, std::bind(&SimulationNode::world_info_callback, this, std::placeholders::_1));
    plan_subscription_ = this->create_subscription<plansys2_msgs::msg::Plan>(
        plan_topic, 10, std::bind(&SimulationNode::plan_callback, this, std::placeholders::_1));

    // Publishers
    progress_publisher_ = this->create_publisher<plansys2_msgs::msg::ActionExecutionInfo>(result_topic, 10);
    knowledge_publisher_ = this->create_publisher<plansys2_msgs::msg::Knowledge>(
        "/simulation_knowledge_" + robot_state_.robot_name, 10);

    // Timer for progress updates
    timer_ = this->create_wall_timer(
        1s, std::bind(&SimulationNode::update_progress, this));

    // Random failure generator
    gen_ = std::mt19937(rd_());
    dist_ = std::uniform_real_distribution<>(0.0, 1.0);
    failure_type_dist_ = std::uniform_int_distribution<>(1, 3);

    // Load failure type config from disk
    std::ifstream failure_file("/tmp/failure_index.json"); 
    if (failure_file.is_open()) {
        try {
            failure_file >> failure_type_config_;
            // RCLCPP_INFO(this->get_logger(), "✅ Loaded failure types from json file");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to parse failure types JSON: %s", e.what());
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ Could not open /tmp/failure_index.json");
    }

    // Logging for initialization
    RCLCPP_INFO(this->get_logger(), "SimulationNode created for robot: %s",
                robot_state_.robot_name.c_str());
}


void SimulationNode::world_info_callback(const plansys2_msgs::msg::WorldInfo::SharedPtr msg)
{
    // Clear existing world state
    world_state_.points.clear();
    world_state_.sites.clear();

    // Populate points from the WorldInfo message
    for (const auto &point : msg->points) {
        world_state_.points[point.id] = point;  // Directly store the Point message
    }

    // Populate sites from the WorldInfo message
    for (const auto &site : msg->sites) {
        world_state_.sites[site.id] = site;  // Directly store the Site message
    }

    RCLCPP_INFO(this->get_logger(), "🌍 WorldInfo updated: %zu points and %zu sites",
            world_state_.points.size(), world_state_.sites.size());

    for (const auto &entry : world_state_.points) {
        RCLCPP_INFO(this->get_logger(), "📌 Loaded POI: %s", entry.first.c_str());
    }
}

float SimulationNode::get_estimated_duration(const std::string &action_name, const std::string &target_name)
{
    // If it's a navigation action, calculate the duration dynamically
    if (action_name == "navigation_air" || action_name == "navigation_water" || action_name == "change_site") {
        // for (const auto &pt : world_state_.points) {
        //     RCLCPP_INFO(this->get_logger(), "We have this point updated: %s and our target is: %s", pt.second.id.c_str(), target_name.c_str());
        // }

        // RCLCPP_INFO(this->get_logger(), "robot at this location: %s, target name is: %s",
        //                     robot_state_.current_location.c_str(), target_name.c_str());
        
        if (world_state_.points.find(robot_state_.current_location) == world_state_.points.end() ||
            world_state_.points.find(target_name) == world_state_.points.end()) {
            RCLCPP_WARN(this->get_logger(), "Invalid location: %s or %s not found in world state",
                        robot_state_.current_location.c_str(), target_name.c_str());
            return -1.0; // Invalid duration
        }

        const auto &current_point = world_state_.points.at(robot_state_.current_location);
        const auto &target_point = world_state_.points.at(target_name);

        // Calculate Euclidean distance between current location and target location
        float distance = std::sqrt(std::pow(target_point.coordinates[0] - current_point.coordinates[0], 2) +
                                   std::pow(target_point.coordinates[1] - current_point.coordinates[1], 2) +
                                   std::pow(target_point.coordinates[2] - current_point.coordinates[2], 2));

        if (action_name == "navigation_air" || action_name == "change_site") {
            return distance / robot_state_.airspeed;
        } else if (action_name == "navigation_water") {
            return distance / robot_state_.waterspeed;
        }
    }

    // If it's an action with a fixed duration, return its predefined duration
    if (action_name == "takeoff") {
        return 4.0; // robot_state_.takeoff_duration;
    } else if (action_name == "landing") {
        return 3.0;
    } else if (action_name == "observe") {
        // Find the current location in the world state
        auto current_poi_it = world_state_.points.find(robot_state_.current_location);
        if (current_poi_it == world_state_.points.end()) {
            RCLCPP_WARN(this->get_logger(), "Invalid current location: %s not found in world state",
                        robot_state_.current_location.c_str());
            return -1.0; // Invalid duration
        }

        // Check which site the current location belongs to
        for (const auto &site_pair : world_state_.sites) {
            const auto &site = site_pair.second;
            for (const auto &poi : site.points) {
                if (poi == robot_state_.current_location) {
                    // Current location found in this site, use its size
                    return site.size / robot_state_.airspeed;
                }
            }
        }

        RCLCPP_WARN(this->get_logger(), "Current location %s does not belong to any site",
                    robot_state_.current_location.c_str());
        return -1.0; // Invalid duration
    } else if (action_name == "observe_2r") {
        // Find the current location in the world state
        auto current_poi_it = world_state_.points.find(robot_state_.current_location);
        if (current_poi_it == world_state_.points.end()) {
            RCLCPP_WARN(this->get_logger(), "Invalid current location: %s not found in world state",
                        robot_state_.current_location.c_str());
            return -1.0; // Invalid duration
        }

        // Check which site the current location belongs to
        for (const auto &site_pair : world_state_.sites) {
            const auto &site = site_pair.second;
            for (const auto &poi : site.points) {
                if (poi == robot_state_.current_location) {
                    // Current location found in this site, use its size
                    return (site.size / robot_state_.airspeed) / 2; // Two robots working together
                }
            }
        }

        RCLCPP_WARN(this->get_logger(), "Current location %s does not belong to any site",
                    robot_state_.current_location.c_str());
        return -1.0; // Invalid duration
    } else if (action_name == "switch_waterair") {
        return 8.0;
    } else if (action_name == "switch_airwater") {
        return 5.0;
    } else if (action_name == "translate_data") {
        return 45.0; // Fixed duration for translating data
    } else if (action_name == "sample") {
        return 30.0; // Fixed duration for sampling
    }

    // If the action is not recognized, return -1.0 as an error
    RCLCPP_WARN(this->get_logger(), "Unknown action type: %s", action_name.c_str());
    return -1.0;
}



std::string SimulationNode::determine_failure_type(const std::string &action_name,const std::string current_action_suffix_)
{
    double random_draw = dist_(gen_);
    // RCLCPP_INFO(this->get_logger(), "🎲 Random draw for forced 50%% failure: %.3f", random_draw);
    // if (random_draw < 0.5){
    //     if (robot_state_.current_action != "change_site")
    //     {
    //         return "1 fail 0";
    //     }        
    // }

    std::string full_action_key = action_name + current_action_suffix_;
    auto it = robot_state_.planned_actions.find(full_action_key);
    if (it != robot_state_.planned_actions.end()) {
        return it->second.failure_type;  // Return the failure type from planned_actions
    }

    // If the action does not exist, perform random failure determination
    if (random_draw < 0.05) {  // ~5% chance of failure
        if (failure_type_config_.contains(action_name)) {
            const auto &failures = failure_type_config_[action_name];
            int max_index = failures.size() - 1;

            if (max_index >= 0) {
                std::uniform_int_distribution<> fail_dist(0, max_index);
                int selected_failure = fail_dist(gen_);

                RCLCPP_INFO(this->get_logger(), "💥 Random failure [%s] → index %d",
                            action_name.c_str(), selected_failure);

                return "1 fail " + std::to_string(selected_failure);
            }
        }

        RCLCPP_WARN(this->get_logger(), "⚠️ No failure entries for action %s", action_name.c_str());
    }

    return "0 none";  // Default to no failure if random failure doesn't trigger
}

void SimulationNode::failure_callback(const plansys2_msgs::msg::Failure::SharedPtr msg)
{
    for (const auto &item : msg->items) {
        auto it = robot_state_.planned_actions.find(item.action);
        if (it != robot_state_.planned_actions.end()) {
            it->second.failure_type = item.failuretype;

            if (item.failuretype.rfind("delay", 0) == 0) {
                float added_delay = 10.0;
                try {
                    std::string delay_str = item.failuretype.substr(6);
                    added_delay = std::stof(delay_str);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "⚠️ Failed to parse delay from '%s'. Using fallback (10s).",
                                item.failuretype.c_str());
                }

                it->second.duration += added_delay;
                if (current_action_id_+current_action_suffix_ == item.action) {
                    robot_state_.current_action_estimated_duration += added_delay;
                    failure_bool_ = "1";
                    failure_type_ = "delay";
                    stored_delay_value_ = added_delay;
                }

                RCLCPP_INFO(this->get_logger(), "💤 Delay of %.2f seconds applied to action [%s]",
                            added_delay, item.action.c_str());
            } else {
                if (current_action_id_+current_action_suffix_ == item.action) {
                    failure_bool_ = "1";
                    failure_type_ = item.failuretype;
                }
                RCLCPP_INFO(this->get_logger(), "⚠️ Action [%s] marked as failed. Type: %s",
                            item.action.c_str(), item.failuretype.c_str());
            }

        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ Could not find action [%s] in planned_actions to apply failure.",
                        item.action.c_str());
        }
    }
}

void SimulationNode::plan_callback(const plansys2_msgs::msg::Plan::SharedPtr msg)
{
    robot_state_.planned_actions.clear();
    std::unordered_map<std::string, int> action_counters;

    // First pass: count how many times each action appears
    for (const auto &item : msg->items) {
        std::string cleaned_action = item.action;
        if (!cleaned_action.empty() && cleaned_action.front() == '(' && cleaned_action.back() == ')') {
            cleaned_action = cleaned_action.substr(1, cleaned_action.size() - 2);
        }
        action_counters[cleaned_action]++;
    }

    // 🔍 Print the counter summary
    RCLCPP_INFO(this->get_logger(), "📊 Action count summary:");
    for (const auto &[action, count] : action_counters) {
        RCLCPP_INFO(this->get_logger(), "  - %s : %d", action.c_str(), count);
    }

    // Second pass: insert with suffixes if needed
    std::unordered_map<std::string, int> current_index;
    for (const auto &item : msg->items) {
        std::string cleaned_action = item.action;
        if (!cleaned_action.empty() && cleaned_action.front() == '(' && cleaned_action.back() == ')') {
            cleaned_action = cleaned_action.substr(1, cleaned_action.size() - 2);
        }

        std::string suffix = "";
        if (action_counters[cleaned_action] > 1) {
            int count = current_index[cleaned_action]++;
            suffix = " " + std::string(1, 'A' + count);  // A, B, C...
        }

        std::string full_action_key = cleaned_action + suffix;
        ActionInfo action_info(full_action_key, "0 none", item.time, item.duration, false);
        robot_state_.planned_actions[full_action_key] = action_info;

        RCLCPP_INFO(this->get_logger(),
                    "📦 Added action '%s' with start time %.2f, duration %.2f, and failure type '%s'",
                    full_action_key.c_str(), item.time, item.duration, "0 none");
    }

    RCLCPP_INFO(this->get_logger(), "Plan received with %zu actions", msg->items.size());
}


void SimulationNode::action_callback(const plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg)
{
    if (msg->status == plansys2_msgs::msg::ActionExecutionInfo::CANCELLED) {
        RCLCPP_INFO(this->get_logger(), "Received cancellation for action: (%s)", msg->action_full_name.c_str());
        action_active_ = false;
        publish_knowledge_update();
        return;
    }

    if (!action_active_ || msg->action_full_name != current_action_id_) {
        
        RCLCPP_INFO(this->get_logger(), "Received new action simulation request: (%s)", msg->action_full_name.c_str());
        
        robot_state_.current_action_progress  = 0.0;
        robot_state_.target = msg->arguments[1];
        robot_state_.current_location = msg->arguments[0];
        robot_state_.current_action =  msg->action;

        RCLCPP_INFO(this->get_logger(), "Current location: %s", robot_state_.current_location.c_str());
        current_action_id_ = msg->action_full_name;
        
        action_active_ = true;
        start_stamp_action_ = msg->start_stamp;

        RCLCPP_INFO(this->get_logger(), "📥 Incoming action_full_name: [%s]", msg->action_full_name.c_str());

        // for (const auto& [key, info] : robot_state_.planned_actions) {
        //     RCLCPP_INFO(this->get_logger(), "🔍 Planned Action Key: [%s] | executed: %s", key.c_str(), info.executed ? "true" : "false");
        // }

        // 🔍 Resolve possible suffix from planned_actions
        current_action_suffix_  = resolve_suffixed_action_suffix(current_action_id_);
        RCLCPP_INFO(this->get_logger(), "current_action_suffix_: %s, of action %s", current_action_suffix_.c_str(), current_action_id_.c_str());

        std::string full_action_key = current_action_id_ + current_action_suffix_;
        RCLCPP_INFO(this->get_logger(), "🎯 Using full key: [%s]", full_action_key.c_str());

        // RCLCPP_INFO(this->get_logger(), "Action is active");

        failure_status_ = determine_failure_type(current_action_id_, current_action_suffix_);

        std::istringstream isstring(failure_status_);
        isstring >> failure_bool_ >> failure_type_;

        RCLCPP_INFO(this->get_logger(), "Will the action fail: %s", failure_status_.c_str());

        robot_state_.current_action_estimated_duration = get_estimated_duration(robot_state_.current_action, robot_state_.target);

    } else {
        RCLCPP_INFO(this->get_logger(), "Action already in progress: %s", current_action_id_.c_str());
    }
}

void SimulationNode::publish_knowledge_update()
{
    plansys2_msgs::msg::Knowledge knowledge_msg;

    knowledge_msg.instances.push_back(robot_state_.robot_name);
    knowledge_msg.instances.push_back(robot_state_.current_location);
    knowledge_msg.predicates.push_back("(robot_at " + robot_state_.robot_name + " " + robot_state_.current_location + ")");
    knowledge_msg.predicates.push_back("(battery_full " + robot_state_.robot_name + ")");

    knowledge_publisher_->publish(knowledge_msg);
}

void SimulationNode::update_progress()
{
    if (!action_active_) {
        return;
    }

    // double now = this->get_clock()->now().seconds();
    // double expected_end = rclcpp::Time(start_stamp_action_).seconds() + robot_state_.current_action_estimated_duration;

    // RCLCPP_INFO(this->get_logger(),
    //     "⏱️ [%s] Real-time expected end: %.2f",
    //     robot_state_.robot_name.c_str(),  expected_end - now);

    // if (robot_state_.planned_actions.count(current_action_id_+current_action_suffix_)) {
    //     const auto& info = robot_state_.planned_actions.at(current_action_id_+current_action_suffix_);
    //     double planned_end = info.time + info.duration;

    //     RCLCPP_INFO(this->get_logger(),
    //         "📆 Plan expected end for [%s]: %.2f (Planner start: %.2f, duration: %.2f)",
    //         current_action_id_.c_str(), planned_end, info.time, info.duration);
    // }

    if (failure_bool_ != "0") {
        if (failure_type_ == "delay") {
            RCLCPP_INFO(this->get_logger(), "Delay in action %s, current progress: %f, estimated duration: %f", robot_state_.current_action.c_str(), robot_state_.current_action_progress, robot_state_.current_action_estimated_duration);
            failure_bool_ = "0";
            failure_type_ = "none";
            stored_delay_value_ = 0.0;
        } else {
            publish_progress(robot_state_.current_action_progress , plansys2_msgs::msg::ActionExecutionInfo::FAILED);
            action_active_ = false;
            return;
        }
    } else {
        robot_state_.current_action_progress += 1.0;
    }

    if (robot_state_.current_action_progress  >= robot_state_.current_action_estimated_duration) {
        std::string full_action_key = current_action_id_ + current_action_suffix_;
        robot_state_.current_action_progress  = robot_state_.current_action_estimated_duration;
        action_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Action %s completed in platform",  robot_state_.current_action.c_str());
        robot_state_.planned_actions[full_action_key].executed = true;
        publish_progress(robot_state_.current_action_progress , plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED);
    } else {
        float progress_percentage = std::round((robot_state_.current_action_progress / robot_state_.current_action_estimated_duration) * 1000) / 10;
        RCLCPP_INFO(this->get_logger(), "Action %s, %s, progress %.1f%%", robot_state_.current_action.c_str(), robot_state_.robot_name.c_str(),progress_percentage);
        publish_progress(robot_state_.current_action_progress, plansys2_msgs::msg::ActionExecutionInfo::EXECUTING);
    }
}

void SimulationNode::save_robot_state_to_world_info(bool failure)
{
    std::ifstream infile("/tmp/world_info.json");
    if (!infile.is_open()) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Could not open /tmp/world_info.json for reading.");
        return;
    }

    nlohmann::json world_json;
    try {
        infile >> world_json;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to parse world_info.json: %s", e.what());
        return;
    }
    infile.close();

    const std::string &start_poi = robot_state_.current_location;
    std::string goal_poi = robot_state_.target;
    if (goal_poi.empty() || world_state_.points.find(goal_poi) == world_state_.points.end()) {
        RCLCPP_DEBUG(this->get_logger(),
            "⚠️ Action target (goal_poi) is missing or invalid [%s]. Falling back to current_location [%s]",
            goal_poi.c_str(), robot_state_.current_location.c_str());
        goal_poi = robot_state_.current_location;
    }

    if (!world_state_.points.count(start_poi) || !world_state_.points.count(goal_poi)) {
        std::string available_pois;
        for (const auto &entry : world_state_.points) {
            available_pois += entry.first + " ";
        }

        RCLCPP_WARN(this->get_logger(), 
            "⚠️ Missing POI(s): '%s' or '%s'\nAvailable POIs: %s", 
            start_poi.c_str(), goal_poi.c_str(), available_pois.c_str());
        return;
    }

    std::vector<float> position(3);
    std::string used_poi;
    std::string inferred_site = "unknown";

    if (failure) {
        // ❌ Failure: keep current state
        position = world_state_.points.at(start_poi).coordinates;
        used_poi = start_poi;
        // 🔍 Retrieve current site from world_info.json in failure mode
        for (const auto &robot : world_json["robots"]) {
            if (robot["name"] == robot_state_.robot_name && robot.contains("site")) {
                inferred_site = robot["site"];
                break;
            }
        }

        RCLCPP_WARN(this->get_logger(), 
            "❌ Robot [%s] action failed. Keeping current POI [%s] and site [%s].",
            robot_state_.robot_name.c_str(), start_poi.c_str(), inferred_site.c_str());
    } else {
        // ✅ Success or in progress: interpolate
        float duration = robot_state_.current_action_estimated_duration;
        float elapsed = robot_state_.current_action_progress;
        float progress = duration > 0.0 ? std::min(elapsed / duration, 1.0f) : 0.0f;

        const auto &start_coords = world_state_.points.at(start_poi).coordinates;
        const auto &goal_coords = world_state_.points.at(goal_poi).coordinates;

        for (size_t i = 0; i < 3; ++i) {
            position[i] = start_coords[i] + progress * (goal_coords[i] - start_coords[i]);
        }

        used_poi = goal_poi;

        for (const auto &[site_id, site] : world_state_.sites) {
            if (std::find(site.points.begin(), site.points.end(), goal_poi) != site.points.end()) {
                inferred_site = site_id;
                break;
            }
        }
    }

    bool updated = false;
    for (auto &robot : world_json["robots"]) {
        if (robot["name"] == robot_state_.robot_name) {
            robot["loc"] = position;
            robot["poi"] = used_poi;
            robot["site"] = inferred_site;
            updated = true;

            RCLCPP_DEBUG(this->get_logger(), "🔄 World info updated for [%s] | poi: %s | site: %s",
                        robot_state_.robot_name.c_str(), used_poi.c_str(), inferred_site.c_str());

            // ✅ Only add missing POI in failure mode
            if (failure && !world_state_.points.count(used_poi)) {
                RCLCPP_WARN(this->get_logger(),
                    "⚠️ POI [%s] not in world_state_.points — adding it due to failure mode.",
                    used_poi.c_str());
                plansys2_msgs::msg::Point new_point;
                new_point.coordinates = position;
                world_state_.points[used_poi] = new_point;
            }

            // 🚫 DO NOT update coordinates of known POIs on success/in-progress
            break;
        }
    }

    if (!updated) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Robot %s not found in world_info.json", robot_state_.robot_name.c_str());
    }

    std::ofstream outfile("/tmp/world_info.json");
    if (outfile.is_open()) {
        outfile << world_json.dump(2);
        // RCLCPP_INFO(this->get_logger(), "📍 Robot [%s] state written to world_info.json", robot_state_.robot_name.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to write updated world_info.json.");
    }
}





void SimulationNode::publish_progress(float completion, int8_t status)
{
    rclcpp::Time now = this->get_clock()->now();
    plansys2_msgs::msg::ActionExecutionInfo progress_msg;

    progress_msg.status = status;
    progress_msg.start_stamp = start_stamp_action_;
    progress_msg.status_stamp = now;
    progress_msg.action_full_name = current_action_id_;
    progress_msg.action = robot_state_.current_action;
    progress_msg.arguments = std::vector<std::string>{robot_state_.current_location};
    progress_msg.completion = completion;
    progress_msg.estimated_duration = robot_state_.current_action_estimated_duration;
    progress_msg.message_status = failure_status_ + current_action_suffix_;

    save_robot_state_to_world_info(status == plansys2_msgs::msg::ActionExecutionInfo::FAILED); // update the state of the robot in worldinfo
    progress_publisher_->publish(progress_msg);

}

std::string SimulationNode::resolve_suffixed_action_suffix(const std::string& base_action)
{
    // 1. Check for exact match (no suffix)
    if (robot_state_.planned_actions.count(base_action) &&
        !robot_state_.planned_actions[base_action].executed)
    {
        RCLCPP_INFO(this->get_logger(),
            "🧠 Resolved base action with NO suffix: [%s]", base_action.c_str());
        return "";
    }

    // 2. Gather suffixed candidates
    std::vector<std::string> candidate_keys;
    for (const auto& [key, info] : robot_state_.planned_actions) {
        if (key.rfind(base_action + " ", 0) == 0 && !info.executed) {
            candidate_keys.push_back(key);
        }
    }

    // 3. Sort keys to prioritize A, B, C...
    std::sort(candidate_keys.begin(), candidate_keys.end());

    if (!candidate_keys.empty()) {
        std::string chosen_key = candidate_keys.front();
        robot_state_.planned_actions[chosen_key].executed = true;
        std::string suffix = chosen_key.substr(base_action.size());  // includes space
        RCLCPP_INFO(this->get_logger(),
            "🧠 Resolved suffix [%s] for base action [%s] → full key: [%s]",
            suffix.c_str(), base_action.c_str(), chosen_key.c_str());
        return suffix;
    }

    RCLCPP_WARN(this->get_logger(),
        "⚠️ No unexecuted match found for [%s] — fallback to empty suffix", base_action.c_str());
    return "";
}