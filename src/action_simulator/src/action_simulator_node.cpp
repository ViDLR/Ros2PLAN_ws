#include "rclcpp/rclcpp.hpp"
#include "action_simulator/msg/action_execution_info.hpp" // Include your custom message
#include <chrono>
#include <random>

using namespace std::literals::chrono_literals;

class SimulationNode : public rclcpp::Node
{
public:
    SimulationNode()
    : Node("simulation_node"), current_progress_(0.0), action_active_(false)
    {
        this->declare_parameter<std::string>("robot_id", "default_robot");
        this->get_parameter("robot_id", robot_id_);  // Initialize robot_id

        std::string info_topic = "/simulation_info_" + robot_id_;
        std::string result_topic = "/simulation_result_" + robot_id_;

        this->subscription_ = this->create_subscription<action_simulator::msg::ActionExecutionInfo>(
            info_topic, 10, std::bind(&SimulationNode::action_callback, this, std::placeholders::_1));

        this->publisher_ = this->create_publisher<action_simulator::msg::ActionExecutionInfo>(result_topic, 10);

        this->timer_ = this->create_wall_timer(
            500ms, std::bind(&SimulationNode::update_progress, this));

        gen_ = std::mt19937(rd_());
        dist_ = std::uniform_real_distribution<>(0.0, 1.0);
    }

private:
    void action_callback(const action_simulator::msg::ActionExecutionInfo::SharedPtr msg)
    {
        if (!action_active_ || msg->action_name != current_action_) {
            RCLCPP_INFO(this->get_logger(), "Received new action simulation request: %s", msg->action_name.c_str());

            // Reset state for new action
            current_action_ = msg->action_name;
            action_active_ = true;
            current_progress_ = 0.0;
            current_action_id_ = msg->action_id;
            failure_ = (dist_(gen_) < 0.01);  // 1% chance of failure
        } else {
            RCLCPP_INFO(this->get_logger(), "Action already in progress: %s", current_action_.c_str());
        }
    }

    void update_progress()
    {
        if (action_active_) {
            if (failure_) {
                RCLCPP_INFO(this->get_logger(), "Action %s failed", current_action_.c_str());
                publish_progress(-1.0);
                action_active_ = false;
            } else {
                current_progress_ += 0.1;
                if (current_progress_ >= 1.0) {
                    current_progress_ = 1.0;
                    action_active_ = false;
                }
                RCLCPP_INFO(this->get_logger(), "Progress for action %s: %f", current_action_.c_str(), current_progress_);
                publish_progress(current_progress_);
            }
        }
    }

    void publish_progress(float progress)
    {
        action_simulator::msg::ActionExecutionInfo progress_msg;
        progress_msg.robot_id = robot_id_;
        progress_msg.action_id = current_action_id_;
        progress_msg.progress = progress;
        progress_msg.action_name = current_action_;
        publisher_->publish(progress_msg);
    }

    std::string current_action_;
    std::string current_action_id_;
    std::string robot_id_;  // Declare robot_id as a member variable
    double current_progress_;
    bool action_active_;
    bool failure_;
    rclcpp::Subscription<action_simulator::msg::ActionExecutionInfo>::SharedPtr subscription_;
    rclcpp::Publisher<action_simulator::msg::ActionExecutionInfo>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dist_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimulationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
