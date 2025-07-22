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
#include <regex>  
#include <sstream> 
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <filesystem>

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
    void ExecutionSequenceFunction(); 
    ~ExecutionManagerNode();

private:
    
    void parseArmsResult(const std::string &file_path,const std::map<std::string, plansys2_msgs::msg::Plan> &labeled_plans);
    std::vector<std::string> splitString(const std::string &input, char delimiter);
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_; // ✅ Declare executor_
    std::thread spin_thread_; // ✅ Declare spin thread
    
    struct ValResult {
        std::string status;  // "SUCCESS" or "FAILURE"
        int value;
        std::vector<std::string> failed_actions;
        std::vector<std::string> repair_advice;
    };

    ValResult parse_val_output(const std::string &file_path);

    // void initializeSTN()
    void load_and_save_world_info(const std::string &problem_info_path);
    void load_and_save_failure_index(const std::string &src_path);
    void publish_world_info(const std::string &file_path);

    // STN handling
    void executionStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    void handleFailure(const std::string &team_name);
    void validateFailureImpact(const std::string& team_name);
    std::map<std::string, std::string> parseFailureStatus(const std::string& msg);
    void applyFailureToProblem(const std::map<std::string, std::string>& failure_data);
    std::shared_ptr<STNController> stn_controller_;
    
    // void reload_knowledge_and_replan();
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr execution_status_sub_;
    rclcpp::Publisher<plansys2_msgs::msg::WorldInfo>::SharedPtr world_info_publisher_;
    std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::vector<plansys2_msgs::msg::Team> active_teams;
    std::map<std::string, plansys2_msgs::msg::Plan> teams_plans;
    std::map<std::string, std::vector<std::string>> team_dependencies_;


};

}  // namespace action_simulator

#endif  // EXECUTION_MANAGER_NODE_HPP_
