#ifndef CHANG_SITE_ACTION_HPP_
#define CHANG_SITE_ACTION_HPP_

#include <memory>
#include <vector>
#include <string>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"

class ChangSiteAction : public plansys2::ActionExecutorClient
{
public:
  ChangSiteAction();

private:
  void do_work() override;
  void result_callback(const plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  bool action_in_progress_;
  std::string current_action_id_;
  std::string robot_id_;
  std::string current_poi1_;
  std::string current_poi2_;
  std::string current_site1_;
  std::string current_site2_;
  std::vector<std::string> specialized_arguments_;
  rclcpp::Publisher<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr publisher_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr subscription_;
};

#endif  // CHANG_SITE_ACTION_HPP_

