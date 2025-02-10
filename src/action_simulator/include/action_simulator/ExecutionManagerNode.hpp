#ifndef EXECUTION_MANAGER_NODE_HPP_
#define EXECUTION_MANAGER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/team.hpp"
#include "plansys2_msgs/srv/start_teams.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "action_simulator/STNControllerNode.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include <plansys2_msgs/msg/world_info.hpp>
#include "plansys2_msgs/msg/action_execution.hpp"
#include <nlohmann/json.hpp>
#include "plansys2_msgs/msg/world_info.hpp"
#include <fmt/core.h>
#include <fmt/ranges.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace action_simulator
{

class ExecutionManagerNode : public rclcpp::Node
{
public:
    ExecutionManagerNode();

private:
    void ExecutionSequenceFunction(); 
    void parseArmsResult(const std::string &file_path, const std::vector<plansys2_msgs::msg::Plan> &plans);
    // void initializeSTN()
    void load_and_save_world_info(const std::string &problem_info_path);
    void publish_world_info(const std::string &file_path);
    void createExecutorCallback(const std::string &team_name);
    void removeExecutorCallback(const std::string &team_name);
    bool hasExecutorCallback(const std::string &team_name) const;
    void addExecutorCallbacks(const std::vector<plansys2_msgs::msg::Team> &teams);

    void addExecutorClients(const std::vector<plansys2_msgs::msg::Team> &teams);
    void createExecutorClient(const std::string &team_name);
    void removeExecutorClient(const std::string &team_name);
    bool hasExecutorClient(const std::string &team_name) const;
    void removeExecutorClients(const std::vector<std::string> &team_names);

    // STN handling
    void startPlanExecution();
    void executionFeedbackCallback(const std::string &team_name, float progress);
    void handleFailure(const std::string &team_name);
    void propagateDelay(const std::string& team_name, float delay);
    void validateFailureImpact(const std::string& team_name);
    std::shared_ptr<STNController> stn_controller_;
    
    // void reload_knowledge_and_replan();
    rclcpp::Publisher<plansys2_msgs::msg::WorldInfo>::SharedPtr world_info_publisher_;
    std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::unordered_map<std::string, rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr> executor_callbacks_;
    std::unordered_map<std::string, std::shared_ptr<plansys2::ExecutorClient>> executor_clients_;
    std::vector<plansys2_msgs::msg::Team> active_teams;
    std::map<std::string, plansys2_msgs::msg::Plan> teams_plans;

    

    


};

}  // namespace action_simulator

#endif  // EXECUTION_MANAGER_NODE_HPP_
