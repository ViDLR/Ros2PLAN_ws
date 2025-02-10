#ifndef STN_CONTROLLER_HPP
#define STN_CONTROLLER_HPP

#include <map>
#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/msg/team.hpp"

class STNController {
public:
    STNController();

    void initializePaths(const std::vector<plansys2_msgs::msg::Team>& teams, 
                         const std::map<std::string, plansys2_msgs::msg::Plan>& plans);

    void trackExecutionProgress(const std::string& team_name, const std::string& site, float progress);
    bool isPathReady(const std::string& team_name);
    void handleFailure(const std::string& team_name);
    void propagateDelay(const std::string& team_name, float delay);

    std::map<std::string, std::vector<std::string>> path_dependencies_;
    std::map<std::string, float> execution_progress_;
    std::map<std::string, std::string> current_site_;

private:
    std::map<std::string, plansys2_msgs::msg::Plan> team_plans_;
    std::map<std::string, bool> team_active_;
    std::map<std::string, float> site_remaining_time_;
};

#endif
