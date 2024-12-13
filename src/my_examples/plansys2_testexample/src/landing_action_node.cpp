#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"

using namespace std::chrono_literals;

class LandingAction : public plansys2::ActionExecutorClient
{
public:
  LandingAction()
  : plansys2::ActionExecutorClient("landing", 1s), action_in_progress_(false)
  {
    this->get_parameter("specialized_arguments", specialized_arguments_);
    
    robot_id_ = specialized_arguments_[0];
    std::string info_topic = "/simulation_info_" + robot_id_;
    std::string result_topic = "/simulation_result_" + robot_id_;

    this->publisher_ = this->create_publisher<plansys2_msgs::msg::ActionExecutionInfo>(info_topic, 10);
    this->subscription_ = this->create_subscription<plansys2_msgs::msg::ActionExecutionInfo>(
        result_topic, 10, std::bind(&LandingAction::result_callback, this, std::placeholders::_1));
  }

private:
  void do_work() override
  {
    if (!action_in_progress_) {
      plansys2_msgs::msg::ActionExecutionInfo msg;
      current_poi1_ = current_arguments_[1];
      msg.action_full_name = action_managed_ + " " + current_poi1_;
      msg.start_stamp = this->get_clock()->now();
      msg.status_stamp = this->get_clock()->now();
      msg.status = plansys2_msgs::msg::ActionExecutionInfo::EXECUTING;
      msg.action = action_managed_ + robot_id_;
      msg.completion = 0.0;
      msg.message_status = ""; // if we give the message of failure it will be here
    
      publisher_->publish(msg);
      action_in_progress_ = true;
      current_action_id_ = action_managed_ + robot_id_;
      RCLCPP_INFO(this->get_logger(), "Sending landing request with action ID: %s and robot ID: %s", current_action_id_.c_str(), robot_id_.c_str());
    }
  }

  void result_callback(const plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg)
  {
    if (msg->action == current_action_id_) {
      float progress = msg->completion;
      int8_t status = msg->status;
      std::string msgstatus = msg-> message_status;
      if (status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
        finish(false, 0.0, "Navigation failed for" + msgstatus);
        action_in_progress_ = false;
        } else if (status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED){
        finish(true, 1.0, "Navigation completed");
        action_in_progress_ = false;
        } else if (status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING){
          if (msgstatus[0] == '1'){
          // The action is delayed but we continue
          send_feedback(progress, "Navigation running based on simulation feedback with a DELAY");
          } else {
          send_feedback(progress, "Navigation running based on simulation feedback");
          }
        }
    }
  }
  
  bool action_in_progress_;
  std::string current_action_id_;
  std::string robot_id_;
  std::string current_poi1_;
  std::vector<std::string> specialized_arguments_;
  rclcpp::Publisher<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr publisher_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr subscription_;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    if (action_in_progress_) {
      plansys2_msgs::msg::ActionExecutionInfo msg;
      msg.action_full_name = action_managed_ + " " + current_poi1_;
      msg.status = plansys2_msgs::msg::ActionExecutionInfo::CANCELLED;
      msg.action = action_managed_ + robot_id_;
      msg.completion = 0.0;
      msg.message_status = ""; // if we give the message of failure it will be here

      publisher_->publish(msg);
      action_in_progress_ = false;
      RCLCPP_INFO(this->get_logger(), "Action Landing with ID: %s and robot ID: %s cancelled", current_action_id_.c_str(), robot_id_.c_str());
    }
    return ActionExecutorClient::on_deactivate(state);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LandingAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "landing"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
