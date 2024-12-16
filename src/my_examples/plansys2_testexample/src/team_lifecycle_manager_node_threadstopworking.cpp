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
        start_teams_service_ = this->create_service<plansys2_msgs::srv::StartTeams>(
            "start_teams",
            std::bind(&TeamLifecycleManager::handleStartTeams, this, std::placeholders::_1, std::placeholders::_2));

        stop_teams_service_ = this->create_service<plansys2_msgs::srv::StopTeams>(
            "stop_teams",
            std::bind(&TeamLifecycleManager::handleStopTeams, this, std::placeholders::_1, std::placeholders::_2));
    }

    ~TeamLifecycleManager()
    {
        stopAllExecutors();
    }

private:
    rclcpp::Service<plansys2_msgs::srv::StartTeams>::SharedPtr start_teams_service_;
    rclcpp::Service<plansys2_msgs::srv::StopTeams>::SharedPtr stop_teams_service_;
    std::map<std::string, std::shared_ptr<std::thread>> team_threads_;
    std::map<std::string, std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>> team_executors_;
    std::mutex thread_mutex_;
    std::atomic<bool> stop_all_flag_;

    void handleStartTeams(
        const plansys2_msgs::srv::StartTeams::Request::SharedPtr request,
        const plansys2_msgs::srv::StartTeams::Response::SharedPtr response)
    {
        for (const auto &team_entry : request->teams)
        {
            const std::string &team_name = team_entry.name;

            std::lock_guard<std::mutex> lock(thread_mutex_);

            if (team_threads_.count(team_name) > 0)
            {
                RCLCPP_WARN(this->get_logger(), "Team %s is already running.", team_name.c_str());
                continue;
            }

            auto exe = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
            auto executor_node = std::make_shared<plansys2::ExecutorNode>();
            exe->add_node(executor_node->get_node_base_interface());

             // Add lifecycle client for the executor
            std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> manager_nodes;
            manager_nodes["executor"] = std::make_shared<plansys2::LifecycleServiceClient>(
                "executor_lc_mngr", "executor");
            // auto lifecycle_client = std::make_shared<plansys2::LifecycleServiceClient>(
            //     "executor_" + team_name + "_lc_mngr", "executor_" + team_name);

            for (auto &manager_node : manager_nodes)
            {
                manager_node.second->init();
                exe->add_node(manager_node.second);
            }

            std::shared_future<bool> startup_future = std::async(
                std::launch::async,
                std::bind(plansys2::startup_function, manager_nodes, std::chrono::seconds(60)));
            exe->spin_until_future_complete(startup_future);
            

            if (!startup_future.get())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to start executor for team: %s", team_name.c_str());
                continue;
            }

            // Store the executor in the map
            team_executors_[team_name] = exe;

            // Create a new thread to spin the executor
            team_threads_[team_name] = std::make_shared<std::thread>([exe]() {
                RCLCPP_INFO(rclcpp::get_logger("team_lifecycle_manager"), "Executor for team spinning.");
                exe->spin();
            });

            RCLCPP_INFO(this->get_logger(), "Started executor for team: %s", team_name.c_str());
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

            if (team_threads_.count(team_name) > 0)
            {
                RCLCPP_INFO(this->get_logger(), "Stopping executor for team: %s", team_name.c_str());

                auto executor = team_executors_[team_name];
                executor->cancel();

                if (team_threads_[team_name]->joinable())
                {
                    team_threads_[team_name]->join();
                }

                team_threads_.erase(team_name);
                team_executors_.erase(team_name);

                RCLCPP_INFO(this->get_logger(), "Stopped executor for team: %s", team_name.c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "Executor for team %s not found or already stopped.",
                            team_name.c_str());
            }
        }

        response->success = true;
        response->message = "Teams stopped successfully.";
    }

    void stopAllExecutors()
    {
        RCLCPP_INFO(this->get_logger(), "Stopping all executors...");

        stop_all_flag_.store(true);

        std::lock_guard<std::mutex> lock(thread_mutex_);
        for (auto &thread_entry : team_threads_)
        {
            const auto &team_name = thread_entry.first;
            auto executor = team_executors_[team_name];
            executor->cancel();

            if (thread_entry.second->joinable())
            {
                thread_entry.second->join();
            }
        }

        team_threads_.clear();
        team_executors_.clear();

        RCLCPP_INFO(this->get_logger(), "All executors stopped.");
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

