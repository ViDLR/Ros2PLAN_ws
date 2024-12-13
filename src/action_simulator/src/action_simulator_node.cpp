#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/knowledge.hpp"
#include <chrono>
#include <random>
#include <iostream>
#include <sstream>
#include <string>

using namespace std::literals::chrono_literals;

class SimulationNode : public rclcpp::Node
{
public:
    SimulationNode()
    : Node("simulation_node"), current_progress_(0.0), action_active_(false)
    {
        this->declare_parameter<std::string>("robot_id", "robot0");
        this->get_parameter("robot_id", robot_id_);  // Initialize robot_id

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
        failure_type_dist_ = std::uniform_int_distribution<>(1, 3); // For selecting failure type
    }

private:
    std::string determine_failure_type() {
        // Check for a failure occurrence
        if (dist_(gen_) < 0.5) { // 1% chance of failure
            int failure_type_gen = failure_type_dist_(gen_);
            return "1 delay";
            // switch (failure_type_gen) {
            //     case 1:
            //         return "1 delay";  // Representing a delay type failure
            //     case 2:
            //         return "1 crash";  // Representing a crash type failure
            //     case 3:
            //         return "1 fail";   // Representing a normal failure
            // }
        }
        return "0 none"; // No failure
    }

    void action_callback(const plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg)
    {
        if (msg->status == plansys2_msgs::msg::ActionExecutionInfo::CANCELLED) {
            RCLCPP_INFO(this->get_logger(), "Received cancellation for action: %s", msg->action_full_name.c_str());
            action_active_ = false;
            publish_knowledge_update();
            return;
        }

        if (!action_active_ || msg->action != current_action_id_) {
            RCLCPP_INFO(this->get_logger(), "Received new action simulation request: %s", msg->action_full_name.c_str());
           
            // Update the state knowledge of the simulator
            std::istringstream iss(msg->action_full_name);
            std::string action, location;
            iss >> action >> location;
            current_poi_ = location;
            RCLCPP_INFO(this->get_logger(), "current location %s", location.c_str());

            // Update action knowledge of the simulator
            current_action_ = msg->action_full_name;
            current_action_id_ = msg->action;
            current_progress_ = 0.0;
            action_active_ = true;
            start_stamp_action_ = msg->start_stamp;
            
            RCLCPP_INFO(this->get_logger(), "action is active");

            // If no instruction to fail has been given, we have a random chance to fail
            if (!msg->message_status.empty() || msg->message_status[0] != '0'){
                failure_status_ = determine_failure_type(); // Determine if action will be a failure and what type
            } else {
                failure_status_ = msg->message_status;
            }
            
            std::istringstream isstring(failure_status_);
            // std::string failure_bool_, failure_type_;
            isstring >> failure_bool_ >> failure_type_;
                       
            RCLCPP_INFO(this->get_logger(), "Will the action fail: %s", failure_status_.c_str());
            
        } else {
            RCLCPP_INFO(this->get_logger(), "Action already in progress: %s", current_action_.c_str());
        }
    }

    void publish_knowledge_update()
    {
        plansys2_msgs::msg::Knowledge knowledge_msg;

        // Example: assuming the robot's position and battery status are tracked
        bool battery_is_low = false; // This should also be dynamically updated

        knowledge_msg.instances.push_back(robot_id_);
        knowledge_msg.instances.push_back(current_poi_);
        knowledge_msg.predicates.push_back("(robot_at " + robot_id_ + " " + current_poi_ + ")");
        if (battery_is_low) {
            knowledge_msg.predicates.push_back("(battery_low " + robot_id_ + ")");
        } else {
            knowledge_msg.predicates.push_back("(battery_full " + robot_id_ + ")");
        }

        state_publisher_->publish(knowledge_msg);
    }

    void update_progress(){
        if (!action_active_) {
            return;
        }
        
        
        // RCLCPP_INFO(this->get_logger(), "Current action: %s, Failure type: %s", current_action_.c_str(), failure_type_.c_str());
        // RCLCPP_INFO(this->get_logger(), "Current action: %f", current_progress_);

        if (failure_bool_ != "0"){
            if (failure_type_ == "delay"){
                // Delays slow down the action progress by halving the normal progress increment
                current_progress_ += 0.05;  // Slower increment for delay
                RCLCPP_INFO(this->get_logger(), "Delay in action %s, current progress: %f", current_action_.c_str(), current_progress_);
                } else if  (failure_type_ == "crash"){
                    // TODO action will be ended at t_x time
                    publish_progress(current_progress_, plansys2_msgs::msg::ActionExecutionInfo::FAILED); // Indicating a failure 
                    action_active_ = false;
                    } else if (failure_type_ == "fail"){
                        publish_progress(current_progress_, plansys2_msgs::msg::ActionExecutionInfo::FAILED); // Indicating a crash
                        action_active_ = false;
                        }
            } else {
                // Normal progress
                current_progress_ += 0.1;
            }

        // Check if the action is complete
        if (current_progress_ >= 1.0) {
            current_progress_ = 1.0;
            action_active_ = false;
            RCLCPP_INFO(this->get_logger(), "Action %s completed in plateform", current_action_.c_str());
            publish_progress(current_progress_, plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED);
        } else{
            publish_progress(current_progress_, plansys2_msgs::msg::ActionExecutionInfo::EXECUTING);
        }    
    }

    void publish_progress(float completion, int8_t status) {
        rclcpp::Time now = this->get_clock()->now();
        plansys2_msgs::msg::ActionExecutionInfo progress_msg;
        progress_msg.status = status;
        progress_msg.start_stamp = start_stamp_action_;
        progress_msg.status_stamp.sec = now.seconds();
        progress_msg.status_stamp.nanosec = now.nanoseconds();
        progress_msg.action_full_name = current_action_;
        progress_msg.action = current_action_id_;
        progress_msg.arguments = {current_poi_};  // Assuming 'current_poi_' holds argument relevant to the action
        progress_msg.duration.sec = static_cast<int>(std::round(10 * (1.0 - completion)));  // Example of dynamic duration based on progress
        progress_msg.completion = completion;
        progress_msg.message_status = failure_status_;
        publish_knowledge_update();
        publisher_->publish(progress_msg);
    }

    std::string current_action_;
    std::string command;
    std::string current_action_id_;
    std::string robot_id_;
    std::string current_poi_;
    std::string current_site_;
    std::string current_configuration_;
    double current_progress_;
    bool action_active_;
    std::string failure_status_;
    std::string failure_type_;
    std::string failure_bool_;
    builtin_interfaces::msg::Time start_stamp_action_;
    rclcpp::Subscription<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr subscription_;
    rclcpp::Publisher<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr publisher_;
    rclcpp::Publisher<plansys2_msgs::msg::Knowledge>::SharedPtr state_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dist_;
    std::uniform_int_distribution<> failure_type_dist_; 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimulationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

