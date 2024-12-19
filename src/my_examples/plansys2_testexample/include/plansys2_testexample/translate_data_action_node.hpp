#ifndef TRANSLATE_DATA_ACTION_NODE_HPP_
#define TRANSLATE_DATA_ACTION_NODE_HPP_

#include <memory>
#include <vector>
#include <string>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TranslateDataActionNode : public plansys2::ActionExecutorClient
{
public:
  TranslateDataActionNode(const std::string &robot_name, const std::string &team_name);

private:
  void do_work() override;
  void result_callback(const plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) override;


  bool action_in_progress_;
  std::string current_action_id_;
  std::string robot_name_;
  std::string current_poi1_;
  std::string current_site1_;
  std::vector<std::string> specialized_arguments_;
  rclcpp::Publisher<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr publisher_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr subscription_;
};

#endif  // TRANSLATE_DATA_ACTION_NODE_



