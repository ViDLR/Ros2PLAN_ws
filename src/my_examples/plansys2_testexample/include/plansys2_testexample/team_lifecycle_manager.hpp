#ifndef TEAM_LIFECYCLE_MANAGER_HPP_
#define TEAM_LIFECYCLE_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "plansys2_msgs/srv/start_teams.hpp"
#include "plansys2_msgs/srv/stop_teams.hpp"
#include <lifecycle_msgs/msg/transition.hpp>
#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"
#include "plansys2_executor/ExecutorNode.hpp"

#include "action_simulator/action_simulator_node.hpp"
#include "plansys2_testexample/change_site_action_node.hpp"
#include "plansys2_testexample/landing_action_node.hpp"
#include "plansys2_testexample/navigation_air_action_node.hpp"
#include "plansys2_testexample/navigation_water_action_node.hpp"
#include "plansys2_testexample/observe_2r_action_node.hpp"
#include "plansys2_testexample/observe_action_node.hpp"
#include "plansys2_testexample/sample_action_node.hpp"
#include "plansys2_testexample/switch_airwater_action_node.hpp"
#include "plansys2_testexample/switch_waterair_action_node.hpp"
#include "plansys2_testexample/takeoff_action_node.hpp"
#include "plansys2_testexample/translate_data_action_node.hpp"

#include <unordered_map>
#include <vector>
#include <map>
#include <string>
#include <mutex>
#include <thread>
#include <memory>
#include <condition_variable>
#include <atomic>

class TeamLifecycleManager : public rclcpp::Node
{
public:
    TeamLifecycleManager();
    ~TeamLifecycleManager();

private:
    enum class ActionTypes
    {
        CHANGE_SITE,
        LANDING,
        NAVIGATION_AIR,
        NAVIGATION_WATER,
        OBSERVE_2R,
        OBSERVE,
        SAMPLE,
        SWITCH_AIRWATER,
        SWITCH_WATERAIR,
        TAKEOFF,
        TRANSLATE_DATA
    };

    void handleStartTeams(
        const plansys2_msgs::srv::StartTeams::Request::SharedPtr request,
        const plansys2_msgs::srv::StartTeams::Response::SharedPtr response);

    void handleStopTeams(
        const plansys2_msgs::srv::StopTeams::Request::SharedPtr request,
        const plansys2_msgs::srv::StopTeams::Response::SharedPtr response);

    void stopAllExecutors();

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> createActionNode(
        ActionTypes action_type,
        const std::string &robot_name,
        const std::string &team_name);

    std::vector<ActionTypes> getAllActionTypes();

    rclcpp::Service<plansys2_msgs::srv::StartTeams>::SharedPtr start_teams_service_;
    rclcpp::Service<plansys2_msgs::srv::StopTeams>::SharedPtr stop_teams_service_;
    std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> lifecycle_clients_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> shared_executor_;
    std::map<std::string, std::shared_ptr<plansys2::ExecutorNode>> team_nodes_;
    std::mutex thread_mutex_;
    std::atomic<bool> stop_all_flag_;
    std::thread spin_thread_;
    std::map<std::string, std::vector<std::shared_ptr<rclcpp::Node>>> normal_robot_nodes_;
    std::map<std::string, std::vector<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>> lifecycle_robot_nodes_;
};

#endif  // TEAM_LIFECYCLE_MANAGER_HPP_
