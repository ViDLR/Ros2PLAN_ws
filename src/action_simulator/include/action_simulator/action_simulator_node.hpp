#ifndef SIMULATION_NODE_HPP_
#define SIMULATION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/knowledge.hpp"
#include <chrono>
#include <random>
#include <string>

class SimulationNode : public rclcpp::Node
{
public:
    SimulationNode(const std::string &robot_id, const std::string &team_name);

private:
    void action_callback(const plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg);
    void update_progress();
    void publish_progress(float completion, int8_t status);
    void publish_knowledge_update();
    std::string determine_failure_type();

    std::string current_action_;
    std::string current_action_id_;
    std::string robot_id_;
    std::string current_poi_;
    std::string failure_status_;
    std::string failure_type_;
    std::string failure_bool_;
    double current_progress_;
    bool action_active_;

    builtin_interfaces::msg::Time start_stamp_action_;
    rclcpp::Subscription<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr subscription_;
    rclcpp::Publisher<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr publisher_;
    rclcpp::Publisher<plansys2_msgs::msg::Knowledge>::SharedPtr state_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dist_;
    std::uniform_int_distribution<> failure_type_dist_;
};

#endif  // SIMULATION_NODE_HPP_

