#include <rclcpp/rclcpp.hpp>
#include <plansys2_msgs/srv/start_teams.hpp>
#include <plansys2_msgs/srv/stop_teams.hpp>
#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
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
    TeamLifecycleManager() : Node("team_lifecycle_manager"), stop_all_flag_(false)
    {
        // Create services for starting and stopping teams
        start_teams_service_ = this->create_service<plansys2_msgs::srv::StartTeams>(
            "start_teams",
            std::bind(&TeamLifecycleManager::handleStartTeams, this, std::placeholders::_1, std::placeholders::_2));
        stop_teams_service_ = this->create_service<plansys2_msgs::srv::StopTeams>(
            "stop_teams",
            std::bind(&TeamLifecycleManager::handleStopTeams, this, std::placeholders::_1, std::placeholders::_2));

        // Initialize the shared MultiThreadedExecutor
        shared_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 4);
        spin_thread_ = std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "Spinning shared executor...");
            shared_executor_->spin();
        });
    }

    ~TeamLifecycleManager()
    {
        stopAllExecutors();
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
    }

private:
    rclcpp::Service<plansys2_msgs::srv::StartTeams>::SharedPtr start_teams_service_;
    rclcpp::Service<plansys2_msgs::srv::StopTeams>::SharedPtr stop_teams_service_;
    std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> lifecycle_clients_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> shared_executor_;
    std::map<std::string, std::shared_ptr<plansys2::ExecutorNode>> team_nodes_;
    std::mutex thread_mutex_;
    std::atomic<bool> stop_all_flag_;
    std::thread spin_thread_;

    void handleStartTeams(
    const plansys2_msgs::srv::StartTeams::Request::SharedPtr request,
    const plansys2_msgs::srv::StartTeams::Response::SharedPtr response)
{
    for (const auto &team_entry : request->teams)
    {
        const std::string &team_name = team_entry.name;

        std::lock_guard<std::mutex> lock(thread_mutex_);
        if (team_nodes_.count(team_name) > 0)
        {
            RCLCPP_WARN(this->get_logger(), "Team %s is already running.", team_name.c_str());
            continue;
        }

        // Create and add a new ExecutorNode for the team
        RCLCPP_INFO(this->get_logger(), "Starting team: %s", team_name.c_str());
        auto team_node = std::make_shared<plansys2::ExecutorNode>("executor_" + team_name, "/" + team_name);
        shared_executor_->add_node(team_node->get_node_base_interface());
        team_nodes_[team_name] = team_node;

        // Add LifecycleServiceClient for the ExecutorNode
        RCLCPP_INFO(this->get_logger(), "Adding LifecycleServiceClient for team: %s", team_name.c_str());
        auto lifecycle_client = std::make_shared<plansys2::LifecycleServiceClient>(
            "executor_" + team_name + "_lc_mngr", "executor_" + team_name, "/" + team_name);
        lifecycle_client->init();
        shared_executor_->add_node(lifecycle_client);
        lifecycle_clients_[team_name] = lifecycle_client;

        // Trigger lifecycle transitions
        RCLCPP_INFO(this->get_logger(), "Configuring LifecycleServiceClient for team: %s", team_name.c_str());
        if (!lifecycle_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure LifecycleServiceClient for team: %s", team_name.c_str());
            continue;
        }

        RCLCPP_INFO(this->get_logger(), "Activating LifecycleServiceClient for team: %s", team_name.c_str());
        if (!lifecycle_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to activate LifecycleServiceClient for team: %s", team_name.c_str());
            continue;
        }

        RCLCPP_INFO(this->get_logger(), "LifecycleServiceClient for team %s successfully configured and activated.", team_name.c_str());
        RCLCPP_INFO(this->get_logger(), "Team %s started.", team_name.c_str());
    }

    response->success = true;
    response->message = "Teams started successfully.";
    }

    void handleStopTeams(
    const plansys2_msgs::srv::StopTeams::Request::SharedPtr request,
    const plansys2_msgs::srv::StopTeams::Response::SharedPtr response)
{
    std::lock_guard<std::mutex> lock(thread_mutex_);
    for (const auto &team_entry : request->teams)
    {
        const std::string &team_name = team_entry.name;

        if (team_nodes_.count(team_name) == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Team %s is not running.", team_name.c_str());
            continue;
        }

        // Deactivate and shut down lifecycle manager
        auto lifecycle_client = lifecycle_clients_[team_name];
        if (lifecycle_client)
        {
            RCLCPP_INFO(this->get_logger(), "Deactivating LifecycleServiceClient for team: %s", team_name.c_str());
            if (!lifecycle_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
            {
                RCLCPP_WARN(this->get_logger(), "Failed to deactivate LifecycleServiceClient for team: %s", team_name.c_str());
            }
            if (!lifecycle_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN))
            {
                RCLCPP_WARN(this->get_logger(), "Failed to shutdown LifecycleServiceClient for team: %s", team_name.c_str());
            }
            shared_executor_->remove_node(lifecycle_client);
            lifecycle_clients_.erase(team_name);
        }

        // Remove the ExecutorNode for the team
        RCLCPP_INFO(this->get_logger(), "Removing ExecutorNode for team: %s", team_name.c_str());
        shared_executor_->remove_node(team_nodes_[team_name]->get_node_base_interface());
        team_nodes_.erase(team_name);

        RCLCPP_INFO(this->get_logger(), "Team %s stopped.", team_name.c_str());
    }

    response->success = true;
    response->message = "Teams stopped successfully.";
    }



    void stopAllExecutors()
    {
        RCLCPP_INFO(this->get_logger(), "Stopping all teams...");
        std::lock_guard<std::mutex> lock(thread_mutex_);
        for (auto &team_entry : team_nodes_)
        {
            shared_executor_->remove_node(team_entry.second->get_node_base_interface());
        }
        team_nodes_.clear();
        RCLCPP_INFO(this->get_logger(), "All teams stopped.");
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeamLifecycleManager>();

    std::thread spin_thread([&]() {
        rclcpp::spin(node);
    });

    RCLCPP_INFO(node->get_logger(), "Team Lifecycle Manager is running.");
    spin_thread.join();

    rclcpp::shutdown();
    return 0;
}