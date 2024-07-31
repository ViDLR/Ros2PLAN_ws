#include <memory>
#include <vector>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "plansys2_msgs/msg/plan.hpp"  // Adjust based on actual message type
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/srv/get_plan.hpp"
#include "std_msgs/msg/color_rgba.hpp"

using namespace std::chrono_literals;

class UserVisualizationNode : public rclcpp::Node
{
public:
  UserVisualizationNode()
  : Node("uservisualization_node")
  {
    // Publisher for Rviz markers
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

    // Timer to update the visualization and request the plan
    timer_ = this->create_wall_timer(500ms, std::bind(&UserVisualizationNode::update_visualization, this));

    // Create a service client for getting the current plan
    get_plan_client_ = this->create_client<plansys2_msgs::srv::GetPlan>("/planner/get_plan");
  }

private:
  void update_visualization()
  {
    // Request the plan from the executor
    auto request = std::make_shared<plansys2_msgs::srv::GetPlan::Request>();
    auto future = get_plan_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = future.get();
      plan_ = response->plan;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service /planner/get_plan");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;

    // Extract actions and robots from the plan
    for (const auto& item : plan_.items) // Adjust based on the actual message structure
    {
      const auto& action = item.action;
      const auto& robot = item.robot; // Adjust based on actual fields
      const auto& start_time = item.start_time; // Adjust based on actual fields
      const auto& duration = item.duration; // Adjust based on actual fields

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "plan";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = start_time.sec; // Adjust based on the actual time field
      marker.pose.position.y = get_robot_y_position(robot); // Assign y position based on robot
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = duration.sec; // Adjust based on actual duration field
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      std_msgs::msg::ColorRGBA color;
      color.r = 1.0;
      color.g = 1.0;
      color.b = 1.0;
      color.a = 1.0;

      marker.color = color;
      marker_array.markers.push_back(marker);
    }

    marker_publisher_->publish(marker_array);
  }

  float get_robot_y_position(const std::string& robot)
  {
    // Map each robot to a unique y position
    static std::map<std::string, float> robot_positions;
    static float current_position = 0.0;

    if (robot_positions.find(robot) == robot_positions.end()) {
      robot_positions[robot] = current_position++;
    }

    return robot_positions[robot];
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Client<plansys2_msgs::srv::GetPlan>::SharedPtr get_plan_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  plansys2_msgs::msg::Plan plan_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UserVisualizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
