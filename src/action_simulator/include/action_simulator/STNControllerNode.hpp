#ifndef STN_CONTROLLER_HPP
#define STN_CONTROLLER_HPP

#include <map>
#include <string>
#include <vector>
#include <memory>
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/msg/plan_item.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/msg/failure.hpp"
#include "plansys2_msgs/msg/failure_item.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"
#include "plansys2_msgs/msg/team.hpp"
#include "plansys2_msgs/srv/start_teams.hpp"
#include "plansys2_msgs/srv/stop_teams.hpp"

using namespace std::chrono_literals;

namespace action_simulator
{

class STNController : public rclcpp::Node
{
public:
    STNController();
    
    void initializePaths(const std::vector<plansys2_msgs::msg::Team>& teams, 
                         const std::map<std::string, plansys2_msgs::msg::Plan>& plans,
                         const std::map<std::string, std::vector<std::string>>& dependencies);

    void publishExecutionStatus();               
    void handleFailure(const std::string& team_name);
    void propagateDelay(const std::string& team_name, float delay);
    void triggerInitialExecutions();
    void startTeamSyncTimer(const std::string& team_name);
    void sendDelayToRobot(const std::string& team_name, const std::string& robot_name, float delay_time);
    std::string extractToken(const std::string& action_string, const std::string& prefix);
    
    // Execution client and callbacks handling handling (Moved from EMN)
    void createExecutorCallback(const std::string &team_name);
    void removeExecutorCallback(const std::string &team_name);
    bool hasExecutorCallback(const std::string &team_name) const;
    
    void createExecutorClient(const std::string &team_name);
    void removeExecutorClient(const std::string &team_name);
    bool hasExecutorClient(const std::string &team_name) const;

    // **Team Lifecycle Management**
    void startTeamExecution(const std::string& team_name);
    void stopTeamExecution(const std::string& team_name);
    void requestTeamCreation(const std::vector<plansys2_msgs::msg::Team>& teams);
    void subscribeToSimulationFeedback(const std::string& robot_name);
    
    struct RobotLiveState {
        std::string current_action;
        rclcpp::Time start_stamp;
        rclcpp::Time status_stamp;
        std::vector<std::string> args;
        double progress;
        double estimated_duration;
        double expected_end_time; // <-- ADD THIS
        int8_t status;
        std::vector<std::string> arguments;
    };

    struct ActionTrackingInfo {
        std::string robot;
        std::string action_name;
        double planned_start_time;
        double planned_duration;
        double expected_end_time;
        std::vector<std::string> depends_on;  // List of action names that must finish before this
    };
    std::map<std::string, std::map<std::string, ActionTrackingInfo>> team_action_tracking_;
    bool isPathReady(const std::string& team_name);
    // void sendDelayToRobot(const std::string& robot_name);
    
    

private:
    std::map<std::string, plansys2_msgs::msg::Plan> team_plans_;
    std::unordered_map<std::string, rclcpp::Publisher<plansys2_msgs::msg::Plan>::SharedPtr> plan_publishers_;
    std::map<std::string, std::vector<std::string>> path_dependencies_;
    
    std::map<std::string, bool> team_active_;
    std::map<std::string, float> execution_progress_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;

    // Executor Clients (Moved from EMN)
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> executor_callbacks_;
    std::unordered_map<std::string, std::shared_ptr<plansys2::ExecutorClient>> executor_clients_;
    std::map<std::string, std::vector<std::string>> reverse_dependencies_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_status_pub_;

    // Team Lifecycle Manager Client
    rclcpp::Client<plansys2_msgs::srv::StartTeams>::SharedPtr start_teams_client_;
    rclcpp::Client<plansys2_msgs::srv::StopTeams>::SharedPtr stop_teams_client_;
    
    std::map<std::string, std::string> robot_to_team_;
    std::map<std::string, std::vector<std::string>> team_to_robots_;
    std::map<std::string, RobotLiveState> robot_live_states_;
    std::map<std::string, rclcpp::Subscription<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr> simulation_result_subs_;
        
    // Delay publishers
    std::map<std::string, rclcpp::Publisher<plansys2_msgs::msg::Failure>::SharedPtr> delay_publishers_;

    // Team timers
    std::map<std::string, rclcpp::TimerBase::SharedPtr> team_sync_timers_;

};

}  // namespace action_simulator

#endif  // STN_CONTROLLER_HPP