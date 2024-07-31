#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "action_simulator/msg/action_execution_info.hpp" // Include your custom message

using namespace std::chrono_literals;

class ChargeAction : public plansys2::ActionExecutorClient
{
public:
  ChargeAction()
  : plansys2::ActionExecutorClient("charge", 1s), action_in_progress_(false)
  {
    this->get_parameter("specialized_arguments", specialized_arguments_);  // Initialize robot_id
    
    robot_id_ = specialized_arguments_[0];
    std::string info_topic = "/simulation_info_" + robot_id_;
    std::string result_topic = "/simulation_result_" + robot_id_;

    this->publisher_ = this->create_publisher<action_simulator::msg::ActionExecutionInfo>(info_topic, 10);
    this->subscription_ = this->create_subscription<action_simulator::msg::ActionExecutionInfo>(
        result_topic, 10, std::bind(&ChargeAction::result_callback, this, std::placeholders::_1));
  }

private:
  void do_work()
  {
    if (!action_in_progress_) {
      action_simulator::msg::ActionExecutionInfo msg;
      msg.robot_id = robot_id_;
      msg.action_id = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
      msg.action_name = "charge";
      msg.progress = 0.0;

      publisher_->publish(msg);
      action_in_progress_ = true;
      current_action_id_ = msg.action_id;
      RCLCPP_INFO(this->get_logger(), "Sending charge request with action ID: %s and robot ID: %s", current_action_id_.c_str(), robot_id_.c_str());
    }
  }

  void result_callback(const action_simulator::msg::ActionExecutionInfo::SharedPtr msg)
  {
    if (msg->action_id == current_action_id_ && msg->robot_id == robot_id_) {
      float progress = msg->progress;
      if (progress < 0.0) {
        finish(false, 0.0, "ChargeAction failed");
        action_in_progress_ = false;
      } else {
        send_feedback(progress, "ChargeAction running based on simulation feedback");
        if (progress >= 1.0) {
          finish(true, 1.0, "ChargeAction completed");
          action_in_progress_ = false;
        }
      }
    }
  }

  bool action_in_progress_;
  std::string current_action_id_;
  std::string robot_id_;  // Declare robot_id as a member variable
  rclcpp::Publisher<action_simulator::msg::ActionExecutionInfo>::SharedPtr publisher_;
  rclcpp::Subscription<action_simulator::msg::ActionExecutionInfo>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChargeAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "charge"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
