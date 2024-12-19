#include "action_simulator/action_simulator_node.hpp"
#include <sstream>

using namespace std::literals::chrono_literals;

SimulationNode::SimulationNode(const std::string &robot_id, const std::string &team_name)
: Node("simulation_node_" + robot_id, team_name),  // Pass namespace to the Node constructor
  robot_id_(robot_id),
  current_progress_(0.0),
  action_active_(false)
{
    // Explicitly declare parameters
    this->declare_parameter<std::string>("robot_id", robot_id);
    this->declare_parameter<std::string>("team_name", team_name);

    // Fetch the parameters to ensure they're set correctly
    this->get_parameter("robot_id", robot_id_);
    this->get_parameter("team_name", team_name_);

    std::string info_topic = "/simulation_info_" + robot_id_;
    std::string result_topic = "/simulation_result_" + robot_id_;

    this->subscription_ = this->create_subscription<plansys2_msgs::msg::ActionExecutionInfo>(
        info_topic, 10, std::bind(&SimulationNode::action_callback, this, std::placeholders::_1));

    this->publisher_ = this->create_publisher<plansys2_msgs::msg::ActionExecutionInfo>(result_topic, 10);

    this->state_publisher_ = this->create_publisher<plansys2_msgs::msg::Knowledge>(
        "/simulation_knowledge_" + robot_id_, 10);

    this->timer_ = this->create_wall_timer(
        500ms, std::bind(&SimulationNode::update_progress, this));

    gen_ = std::mt19937(rd_());
    dist_ = std::uniform_real_distribution<>(0.0, 1.0);
    failure_type_dist_ = std::uniform_int_distribution<>(1, 3);

    RCLCPP_INFO(this->get_logger(), "SimulationNode created for robot: %s in team: %s", robot_id_.c_str(), team_name_.c_str());
}


std::string SimulationNode::determine_failure_type()
{
    if (dist_(gen_) < 0.5) {
        int failure_type_gen = failure_type_dist_(gen_);
        // return "1 delay";
        switch (failure_type_gen) {
            case 1:
                return "1 delay";  // Representing a delay type failure
            case 2:
                return "1 crash";  // Representing a crash type failure
            case 3:
                return "1 fail";   // Representing a normal failure
        }
    }
    return "0 none";
}


void SimulationNode::action_callback(const plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg)
{
    if (msg->status == plansys2_msgs::msg::ActionExecutionInfo::CANCELLED) {
        RCLCPP_INFO(this->get_logger(), "Received cancellation for action: %s", msg->action_full_name.c_str());
        action_active_ = false;
        publish_knowledge_update();
        return;
    }

    if (!action_active_ || msg->action != current_action_id_) {
        RCLCPP_INFO(this->get_logger(), "Received new action simulation request: %s", msg->action_full_name.c_str());
        std::istringstream iss(msg->action_full_name);
        std::string action, location;
        iss >> action >> location;
        current_poi_ = location;

        RCLCPP_INFO(this->get_logger(), "Current location: %s", location.c_str());

        current_action_ = msg->action_full_name;
        current_action_id_ = msg->action;
        current_progress_ = 0.0;
        action_active_ = true;
        start_stamp_action_ = msg->start_stamp;

        RCLCPP_INFO(this->get_logger(), "Action is active");

        failure_status_ = msg->message_status.empty() || msg->message_status[0] != '0'
                          ? determine_failure_type()
                          : msg->message_status;

        std::istringstream isstring(failure_status_);
        isstring >> failure_bool_ >> failure_type_;

        RCLCPP_INFO(this->get_logger(), "Will the action fail: %s", failure_status_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Action already in progress: %s", current_action_.c_str());
    }
}

void SimulationNode::publish_knowledge_update()
{
    plansys2_msgs::msg::Knowledge knowledge_msg;

    knowledge_msg.instances.push_back(robot_id_);
    knowledge_msg.instances.push_back(current_poi_);
    knowledge_msg.predicates.push_back("(robot_at " + robot_id_ + " " + current_poi_ + ")");
    knowledge_msg.predicates.push_back("(battery_full " + robot_id_ + ")");

    state_publisher_->publish(knowledge_msg);
}

void SimulationNode::update_progress()
{
    if (!action_active_) {
        return;
    }

    if (failure_bool_ != "0") {
        if (failure_type_ == "delay") {
            current_progress_ += 0.05;
            RCLCPP_INFO(this->get_logger(), "Delay in action %s, current progress: %f", current_action_.c_str(), current_progress_);
        } else {
            publish_progress(current_progress_, plansys2_msgs::msg::ActionExecutionInfo::FAILED);
            action_active_ = false;
            return;
        }
    } else {
        current_progress_ += 0.1;
    }

    if (current_progress_ >= 1.0) {
        current_progress_ = 1.0;
        action_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Action %s completed in platform", current_action_.c_str());
        publish_progress(current_progress_, plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED);
    } else {
        publish_progress(current_progress_, plansys2_msgs::msg::ActionExecutionInfo::EXECUTING);
    }
}

void SimulationNode::publish_progress(float completion, int8_t status)
{
    rclcpp::Time now = this->get_clock()->now();
    plansys2_msgs::msg::ActionExecutionInfo progress_msg;

    progress_msg.status = status;
    progress_msg.start_stamp = start_stamp_action_;
    progress_msg.status_stamp = now;
    progress_msg.action_full_name = current_action_;
    progress_msg.action = current_action_id_;
    progress_msg.arguments = std::vector<std::string>{current_poi_};
    progress_msg.completion = completion;
    progress_msg.message_status = failure_status_;

    publisher_->publish(progress_msg);
}
