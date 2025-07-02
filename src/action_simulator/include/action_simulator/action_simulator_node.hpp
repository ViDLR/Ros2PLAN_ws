#ifndef SIMULATION_NODE_HPP_
#define SIMULATION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/knowledge.hpp"
#include "plansys2_msgs/msg/failure.hpp"
#include "plansys2_msgs/msg/failure_item.hpp"
#include "plansys2_msgs/msg/world_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include <chrono>
#include <random>
#include <string>
#include <vector>
#include <unordered_map>

class SimulationNode : public rclcpp::Node
{
public:
    SimulationNode(const std::string &robot_name, const std::string &team_name);

private:

    struct ActionInfo {
        std::string name;         // Action name
        std::string failure_type; // Failure type (e.g., "0 none", "1 delay", etc.)
        float time;               // Scheduled start time
        float duration;           // Expected duration

        // Constructor for explicit initialization
        ActionInfo(const std::string &action_name = "",
                const std::string &failure = "0 none",
                float start_time = 0.0,
                float action_duration = 0.0)
            : name(action_name), failure_type(failure), time(start_time), duration(action_duration) {}
    };

    // Robot state representation
    struct RobotState {
        std::string robot_name;
        std::string team;
        std::string current_action;
        std::string current_location;
        int8_t current_configuration = 0;
        std::vector<std::string> capabilities;  // robots capabilities
        float waterspeed = 0.5;
        float airspeed = 1.5;
        float battery_level = 100.0;  // Default to full charge
        float plan_progress = 0.0;
        float current_action_progress = 0.0;
        float current_action_estimated_duration = 0.0;
        std::unordered_map<std::string, ActionInfo> planned_actions;  // Store ActionInfo

    };

    struct WorldState {
        std::unordered_map<std::string, plansys2_msgs::msg::Point> points;  // Point name -> Point
        std::unordered_map<std::string, plansys2_msgs::msg::Site> sites;    // Site name -> Site
    };

    // Callback functions
    void action_callback(const plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg);
    void failure_callback(const plansys2_msgs::msg::Failure::SharedPtr msg);
    void world_info_callback(const plansys2_msgs::msg::WorldInfo::SharedPtr msg);
    void plan_callback(const plansys2_msgs::msg::Plan::SharedPtr msg);

    // Core functionality
    void update_progress();
    float get_estimated_duration(const std::string &action_name, const std::string &target_name);
    void publish_progress(float completion, int8_t status);
    void publish_knowledge_update();
    std::string determine_failure_type(const std::string &action_name,const std::string current_action_suffix_);
    std::string resolve_suffixed_action_suffix(const std::string& base_action);

    // Member variables
    RobotState robot_state_;
    WorldState world_state_;
    std::string current_action_suffix_;

    // Stocking the failure output
    std::string failure_status_;
    std::string failure_type_;
    std::string failure_bool_;
    bool action_active_ = false;
    std::string current_action_id_;
    float stored_delay_value_ = 0.0;

    builtin_interfaces::msg::Time start_stamp_action_;
    std::map<std::string, int> seen_action_counter_;

    // Subscriptions and publishers
    rclcpp::Subscription<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr action_subscription_;
    rclcpp::Subscription<plansys2_msgs::msg::Failure>::SharedPtr failure_subscription_;
    rclcpp::Subscription<plansys2_msgs::msg::WorldInfo>::SharedPtr world_info_subscription_;
    rclcpp::Subscription<plansys2_msgs::msg::Plan>::SharedPtr plan_subscription_;
    rclcpp::Publisher<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr progress_publisher_;
    rclcpp::Publisher<plansys2_msgs::msg::Knowledge>::SharedPtr knowledge_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Random failure generation
    std::random_device rd_;
    std::mt19937 gen_{rd_()};
    std::uniform_real_distribution<> dist_{0.0, 1.0};
    std::uniform_int_distribution<> failure_type_dist_{1, 3};
};

#endif  // SIMULATION_NODE_HPP_
