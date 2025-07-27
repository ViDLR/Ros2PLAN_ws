#include "plansys2_testexample/sample_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string robot_name = "robot0";
  std::string team_name = "";
  auto temp_node = std::make_shared<rclcpp::Node>("arg_parser");
  // Declare both params
  temp_node->declare_parameter("specialized_arguments", std::vector<std::string>{robot_name});
  temp_node->declare_parameter("team_name", team_name);

  // Extract team_name
  temp_node->get_parameter("team_name", team_name);

  // Extract robot_name from specialized_arguments[0]
  std::vector<std::string> args;
  temp_node->get_parameter("specialized_arguments", args);
  if (!args.empty()) {
    robot_name = args[0];
  }
  
  rclcpp::shutdown();
  rclcpp::init(argc, argv);
  auto action_node = std::make_shared<SampleActionNode>(robot_name, team_name);
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(action_node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

