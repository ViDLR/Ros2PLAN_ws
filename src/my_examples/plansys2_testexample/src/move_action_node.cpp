#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "action_simulator/msg/action_execution_info.hpp" // Include your custom message

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move", 1s), action_in_progress_(false)
  {
    this->get_parameter("specialized_arguments", specialized_arguments_);  // Initialize robot_id
    
    robot_id_ = specialized_arguments_[0];
    std::string info_topic = "/simulation_info_" + robot_id_;
    std::string result_topic = "/simulation_result_" + robot_id_;

    this->publisher_ = this->create_publisher<action_simulator::msg::ActionExecutionInfo>(info_topic, 10);
    this->subscription_ = this->create_subscription<action_simulator::msg::ActionExecutionInfo>(
        result_topic, 10, std::bind(&MoveAction::result_callback, this, std::placeholders::_1));
  }

private:
  void do_work() override
  {
    if (!action_in_progress_) {
      action_simulator::msg::ActionExecutionInfo msg;
      msg.robot_id = robot_id_;
      current_room_ = current_arguments_[1];
      goal_room_ = current_arguments_[2];
      msg.action_id = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
      msg.action_name = "move " + current_room_;
      msg.progress = 0.0;

      publisher_->publish(msg);
      action_in_progress_ = true;
      current_action_id_ = msg.action_id;
      RCLCPP_INFO(this->get_logger(), "Sending move request with action ID: %s and robot ID: %s", current_action_id_.c_str(), robot_id_.c_str());
    }
  }

  void result_callback(const action_simulator::msg::ActionExecutionInfo::SharedPtr msg)
  {
    if (msg->action_id == current_action_id_ && msg->robot_id == robot_id_) {
      float progress = msg->progress;
      if (progress < 0.0) {
        finish(false, 0.0, "MoveAction failed");
        action_in_progress_ = false;
      } else {
        send_feedback(progress, "MoveAction running based on simulation feedback");
        if (progress >= 1.0) {
          finish(true, 1.0, "MoveAction completed");
          action_in_progress_ = false;
        }
      }
    }
  }

  bool action_in_progress_;
  std::string current_action_id_;
  std::string robot_id_;
  std::string current_room_;
  std::string goal_room_;
  std::vector<std::string> specialized_arguments_;
  rclcpp::Publisher<action_simulator::msg::ActionExecutionInfo>::SharedPtr publisher_;
  rclcpp::Subscription<action_simulator::msg::ActionExecutionInfo>::SharedPtr subscription_;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    if (action_in_progress_) {
      action_simulator::msg::ActionExecutionInfo msg;
      msg.robot_id = robot_id_;
      msg.action_id = current_action_id_;
      msg.action_name = "move";
      msg.progress = -1.0;  // Indicate cancellation
      publisher_->publish(msg);
      action_in_progress_ = false;
      RCLCPP_INFO(this->get_logger(), "Action move with ID: %s and robot ID: %s cancelled", current_action_id_.c_str(), robot_id_.c_str());
    }
    return ActionExecutorClient::on_deactivate(state);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}