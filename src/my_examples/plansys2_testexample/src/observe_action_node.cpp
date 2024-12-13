#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"

using namespace std::chrono_literals;

class ObserveAction : public plansys2::ActionExecutorClient
{
public:
  ObserveAction()
  : plansys2::ActionExecutorClient("observe_site", 1s), action_in_progress_(false)
  {
    this->get_parameter("specialized_arguments", specialized_arguments_);
    
    robot_id_ = specialized_arguments_[0];
    std::string info_topic = "/simulation_info_" + robot_id_;
    std::string result_topic = "/simulation_result_" + robot_id_;

    this->publisher_ = this->create_publisher<plansys2_msgs::msg::ActionExecutionInfo>(info_topic, 10);
    this->subscription_ = this->create_subscription<plansys2_msgs::msg::ActionExecutionInfo>(
        result_topic, 10, std::bind(&ObserveAction::result_callback, this, std::placeholders::_1));
  }

private:
  void do_work() override
  {
    if (!action_in_progress_) {
      plansys2_msgs::msg::ActionExecutionInfo msg;
      msg.start_stamp = this->get_clock()->now();
      msg.status_stamp = this->get_clock()->now();
      msg.status = plansys2_msgs::msg::ActionExecutionInfo::EXECUTING;
      msg.completion = 0.0;
      msg.message_status = ""; // if we give the message of failure it will be here

      if (current_arguments_.size() == 5) {
        current_poi1_ = current_arguments_[3];
        current_site1_ = current_arguments_[4];
      } else {
        current_poi1_ = current_arguments_[2];
        current_site1_ = current_arguments_[3];
        
      }
      msg.action_full_name = current_arguments_[0] + " " + current_poi1_ + " " + current_site1_;
      msg.action = current_arguments_[0] + robot_id_;
      publisher_->publish(msg);
      action_in_progress_ = true;
      current_action_id_ = current_arguments_[0] + robot_id_;
      RCLCPP_INFO(this->get_logger(), "Sending observe_site request with action ID: %s and robot ID: %s", current_action_id_.c_str(), robot_id_.c_str());
    }
  }

  void result_callback(const plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg)
  {
    if (msg->action == current_action_id_) {
      float progress = msg->completion;
      int8_t status = msg->status;
      std::string msgstatus = msg-> message_status;
      if (status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
        finish(false, 0.0, "observe_site failed for" + msgstatus);
        action_in_progress_ = false;
        } else if (status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED){
        finish(true, 1.0, "observe_site completed");
        action_in_progress_ = false;
        } else if (status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING){
          if (msgstatus[0] == '1'){
          // The action is delayed but we continue
          send_feedback(progress, "observe_site running based on simulation feedback with a DELAY");
          } else {
          send_feedback(progress, "observe_site running based on simulation feedback");
          }
        }
    }
  }

  bool action_in_progress_;
  std::string current_action_id_;
  std::string robot_id_;
  std::string current_poi1_;
  std::string current_site1_;
  std::vector<std::string> specialized_arguments_;
  rclcpp::Publisher<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr publisher_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr subscription_;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    if (action_in_progress_) {
      plansys2_msgs::msg::ActionExecutionInfo msg;
      msg.action_full_name = current_arguments_[0] + " " + current_poi1_;
      msg.status = plansys2_msgs::msg::ActionExecutionInfo::CANCELLED;
      msg.action = current_arguments_[0] + robot_id_;
      msg.completion = 0.0;
      msg.message_status = ""; 

      publisher_->publish(msg);
      action_in_progress_ = false;
      RCLCPP_INFO(this->get_logger(), "Action observe_site with ID: %s and robot ID: %s cancelled", current_action_id_.c_str(), robot_id_.c_str());
    }
    return ActionExecutorClient::on_deactivate(state);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObserveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "observe_site"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
