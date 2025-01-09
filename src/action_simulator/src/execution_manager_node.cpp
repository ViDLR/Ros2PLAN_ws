#include <memory>

#include "action_simulator/ExecutionManagerNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<action_simulator::ExecutionManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
