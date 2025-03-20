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
#include "plansys2_msgs/msg/action_execution.hpp"
#include "plansys2_msgs/msg/team.hpp"
#include "plansys2_msgs/srv/start_teams.hpp"


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
    plansys2_msgs::msg::Plan getStaticPlan();

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

    bool isPathReady(const std::string& team_name);
    

private:
    std::map<std::string, plansys2_msgs::msg::Plan> team_plans_;
    std::map<std::string, std::vector<std::string>> path_dependencies_;
    std::map<std::string, bool> team_active_;
    std::map<std::string, float> execution_progress_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;

    // Executor Clients (Moved from EMN)
    std::unordered_map<std::string, rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr> executor_callbacks_;
    std::unordered_map<std::string, std::shared_ptr<plansys2::ExecutorClient>> executor_clients_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_status_pub_;

    // Team Lifecycle Manager Client
    rclcpp::Client<plansys2_msgs::srv::StartTeams>::SharedPtr start_teams_client_;

};

}  // namespace action_simulator

#endif  // STN_CONTROLLER_HPP