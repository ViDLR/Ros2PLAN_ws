#include "plansys2_testexample/observe_2r_action_node.hpp"
#include <algorithm>

using namespace std::chrono_literals;

Observe2rActionNode::Observe2rActionNode(const std::string &robot_name, const std::string &team_name)
: plansys2::ActionExecutorClient(
      "observe_2r_action_node_" + robot_name,  // Node name
      team_name,                                // Namespace
      1s),                                      // Execution rate
  robot_name_(robot_name),
  action_in_progress_(false)
{
  // Declare parameters
  declare_parameter<std::string>("robot_name", robot_name_);
  declare_parameter<std::string>("team_name", team_name);
}

CallbackReturnT Observe2rActionNode::on_configure(const rclcpp_lifecycle::State & state)
{
  // Call base class on_configure
  auto base_result = ActionExecutorClient::on_configure(state);
  if (base_result != CallbackReturnT::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Base ActionExecutorClient::on_configure failed!");
    return base_result;  // Propagate failure
  }

  std::string info_topic = "/simulation_info_" + robot_name_;
  std::string result_topic = "/simulation_result_" + robot_name_;

  publisher_ = this->create_publisher<plansys2_msgs::msg::ActionExecutionInfo>(info_topic, 10);
  subscription_ = this->create_subscription<plansys2_msgs::msg::ActionExecutionInfo>(
      result_topic, 10, std::bind(&Observe2rActionNode::result_callback, this, std::placeholders::_1));

  return CallbackReturnT::SUCCESS;
}



void Observe2rActionNode::do_work()
{
  if (!action_in_progress_) {
    plansys2_msgs::msg::ActionExecutionInfo msg;
    current_poi1_ = current_arguments_[2];
    current_site1_ = current_arguments_[3];
    msg.action_full_name = action_managed_ + " " + current_poi1_;
    msg.start_stamp = this->get_clock()->now();
    msg.status_stamp = this->get_clock()->now();
    msg.status = plansys2_msgs::msg::ActionExecutionInfo::EXECUTING;
    msg.action = action_managed_ + robot_name_;
    msg.completion = 0.0;
    msg.message_status = ""; // if we give the message of failure it will be here

    publisher_->publish(msg);
    action_in_progress_ = true;
    current_action_id_ = action_managed_ + robot_name_;
    RCLCPP_INFO(this->get_logger(), "Sending observation request with action ID: %s and robot ID: %s", current_action_id_.c_str(), robot_name_.c_str());
  }
}

void Observe2rActionNode::result_callback(const plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg)
{
  if (msg->action == current_action_id_) {
    float progress = msg->completion;
    int8_t status = msg->status;
    std::string msgstatus = msg->message_status;
    if (status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
      finish(false, 0.0, "Observation failed for" + msgstatus);
      action_in_progress_ = false;
    } else if (status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED) {
      finish(true, 1.0, "Observation completed");
      action_in_progress_ = false;
    } else if (status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING) {
      if (msgstatus[0] == '1') {
        // The action is delayed but we continue
        send_feedback(progress, "Observation running based on simulation feedback with a DELAY");
      } else {
        send_feedback(progress, "Observation running based on simulation feedback");
      }
    }
  }
}

CallbackReturnT Observe2rActionNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  // Call base class on_configure
  auto base_result = ActionExecutorClient::on_deactivate(state);
  if (base_result != CallbackReturnT::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Base ActionExecutorClient::on_configure failed!");
    return base_result;  // Propagate failure
  }
  if (action_in_progress_) {
    plansys2_msgs::msg::ActionExecutionInfo msg;
    msg.action_full_name = action_managed_ + " " + current_poi1_;
    msg.status = plansys2_msgs::msg::ActionExecutionInfo::CANCELLED;
    msg.action = current_action_id_;
    msg.completion = 0.0;
    msg.message_status = "";
    publisher_->publish(msg);
    action_in_progress_ = false;
    RCLCPP_INFO(this->get_logger(), "Action observe_2r with ID: %s and robot ID: %s cancelled", current_action_id_.c_str(), robot_name_.c_str());
  }
  return ActionExecutorClient::on_deactivate(state);
}