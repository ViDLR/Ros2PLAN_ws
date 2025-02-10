#include "action_simulator/STNControllerNode.hpp"

STNController::STNController() {}

void STNController::initializePaths(const std::vector<plansys2_msgs::msg::Team>& teams, 
                                 const std::map<std::string, plansys2_msgs::msg::Plan>& plans) {
    team_plans_ = plans;

    for (const auto &team : teams) {
        execution_progress_[team.name] = 0.0;
        team_active_[team.name] = false;
        current_site_[team.name] = "base";

        if (path_dependencies_.count(team.name) == 0) {
            path_dependencies_[team.name] = {};
        }

        for (const auto &action : plans.at(team.name).items) {
            site_remaining_time_[action.action] = action.duration;
        }
    }
}

bool STNController::isPathReady(const std::string& team_name) {
    if (team_active_[team_name]) return false;

    for (const auto& dep : path_dependencies_[team_name]) {
        if (execution_progress_[dep] < 1.0) return false;
    }
    return true;
}

void STNController::trackExecutionProgress(const std::string& team_name, const std::string& site, float progress) {
    execution_progress_[team_name] = progress;
    current_site_[team_name] = site;
}

void STNController::handleFailure(const std::string& team_name) {
    RCLCPP_WARN(rclcpp::get_logger("STNController"), "Failure detected in team: %s", team_name.c_str());

    team_active_[team_name] = false;

    for (auto &entry : path_dependencies_) {
        auto &deps = entry.second;
        if (std::find(deps.begin(), deps.end(), team_name) != deps.end()) {
            execution_progress_[entry.first] = 0.0;
        }
    }
}

void STNController::propagateDelay(const std::string& team_name, float delay) {
    RCLCPP_WARN(rclcpp::get_logger("STNController"), "Delay in team: %s by %f seconds", team_name.c_str(), delay);

    for (const auto &dep : path_dependencies_[team_name]) {
        site_remaining_time_[dep] += delay;
    }
}
