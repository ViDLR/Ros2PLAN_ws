// #include "rclcpp/rclcpp.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"
// #include "plansys2_msgs/msg/plan.hpp"
// #include "plansys2_msgs/msg/action_execution_info.hpp"
// #include "std_msgs/msg/color_rgba.hpp"
// #include "matplotlibcpp.h"
// #include <memory>
// #include <vector>
// #include <string>
// #include <map>
// #include <mutex>
// #include <thread>
// #include <atomic>
// #include <chrono>
// #include <graphviz/cgraph.h>
// #include <graphviz/gvc.h>
// #include <cstdlib>

// namespace plt = matplotlibcpp;

// class UserVisualizationNode : public rclcpp::Node
// {
// public:
//   UserVisualizationNode()
//   : Node("uservisualization_node"), stop_requested_(false), last_plan_size_(0)
//   {
//     RCLCPP_INFO(this->get_logger(), "Initializing UserVisualizationNode...");

//     // Subscriber for the plan
//     plan_subscription_ = this->create_subscription<plansys2_msgs::msg::Plan>(
//       "/executing_plan", 10, std::bind(&UserVisualizationNode::plan_callback, this, std::placeholders::_1));

//     // Setup signal handling for proper shutdown
//     signal(SIGINT, signal_handler);

//     RCLCPP_INFO(this->get_logger(), "UserVisualizationNode initialized.");
//   }

//   ~UserVisualizationNode()
//   {
//     stop_requested_.store(true);
//     if (worker_thread_.joinable()) {
//       worker_thread_.join();
//     }
//   }

//   void init()
//   {
//     RCLCPP_INFO(this->get_logger(), "Starting worker thread...");
//     worker_thread_ = std::thread(&UserVisualizationNode::worker_function, this);
//   }

// private:
//   void plan_callback(const plansys2_msgs::msg::Plan::SharedPtr msg)
//   {
//     RCLCPP_INFO(this->get_logger(), "Received new plan with %zu items.", msg->items.size());
//     std::lock_guard<std::mutex> lock(mutex_);
//     if (msg->items.size() != last_plan_size_) {
//       plan_ = *msg;
//       last_plan_size_ = msg->items.size();
//       plot_plan();
//     } else {
//       RCLCPP_INFO(this->get_logger(), "Plan has not changed, not updating.");
//     }
//   }

//   void plot_plan()
//   {
//     RCLCPP_INFO(this->get_logger(), "Plotting plan...");

//     Agraph_t *g = agopen(const_cast<char *>("PlanGraph"), Agdirected, nullptr);

//     std::map<std::string, int> robot_positions;
//     int robot_pos = 0;

//     for (const auto& item : plan_.items) {
//       const auto& action = item.action;
//       double start_time = item.time;
//       double end_time = item.time + item.duration;

//       RCLCPP_INFO(this->get_logger(), "Action: %s, Start Time: %.2f, End Time: %.2f", action.c_str(), start_time, end_time);

//       std::string robot = action.substr(1, action.find(' ') - 1); // Extract robot name

//       if (robot_positions.find(robot) == robot_positions.end()) {
//         robot_positions[robot] = robot_pos++;
//       }

//       Agnode_t *start_node = agnode(g, const_cast<char *>(std::to_string(start_time).c_str()), TRUE);
//       Agnode_t *end_node = agnode(g, const_cast<char *>(std::to_string(end_time).c_str()), TRUE);
//       agedge(g, start_node, end_node, nullptr, TRUE);
//     }

//     GVC_t *gvc = gvContext();
//     gvLayout(gvc, g, "dot");
//     gvRenderFilename(gvc, g, "png", "plan_graph.png");
//     gvFreeLayout(gvc, g);
//     agclose(g);
//     gvFreeContext(gvc);

//     // Open the graph image using xdg-open
//     system("xdg-open plan_graph.png");

//     RCLCPP_INFO(this->get_logger(), "Plan graph plotted and saved as plan_graph.png");
//   }

//   void worker_function()
//   {
//     while (!stop_requested_.load()) {
//       std::this_thread::sleep_for(std::chrono::seconds(1));
//     }
//   }

//   static void signal_handler(int signal)
//   {
//     if (signal == SIGINT) {
//       rclcpp::shutdown();
//     }
//   }

//   rclcpp::Subscription<plansys2_msgs::msg::Plan>::SharedPtr plan_subscription_;
//   std::mutex mutex_;
//   plansys2_msgs::msg::Plan plan_;
//   std::thread worker_thread_;
//   std::atomic<bool> stop_requested_;
//   size_t last_plan_size_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<UserVisualizationNode>();

//   node->init();
//   rclcpp::spin(node);
//   rclcpp::shutdown();

//   return 0;
// }


// #include "rclcpp/rclcpp.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"
// #include "plansys2_msgs/msg/plan.hpp"
// #include "plansys2_msgs/msg/action_execution_info.hpp"
// #include "std_msgs/msg/color_rgba.hpp"
// #include "matplotlibcpp.h"
// #include <memory>
// #include <vector>
// #include <string>
// #include <map>
// #include <mutex>
// #include <thread>
// #include <atomic>
// #include <chrono>
// #include <graphviz/cgraph.h>
// #include <graphviz/gvc.h>
// #include <cstdlib>

// namespace plt = matplotlibcpp;

// class UserVisualizationNode : public rclcpp::Node
// {
// public:
//   UserVisualizationNode()
//   : Node("uservisualization_node"), stop_requested_(false), last_plan_size_(0)
//   {
//     RCLCPP_INFO(this->get_logger(), "Initializing UserVisualizationNode...");

//     // Subscriber for the plan
//     plan_subscription_ = this->create_subscription<plansys2_msgs::msg::Plan>(
//       "/executing_plan", 10, std::bind(&UserVisualizationNode::plan_callback, this, std::placeholders::_1));

//     // Setup signal handling for proper shutdown
//     signal(SIGINT, signal_handler);

//     RCLCPP_INFO(this->get_logger(), "UserVisualizationNode initialized.");
//   }

//   ~UserVisualizationNode()
//   {
//     stop_requested_.store(true);
//     if (worker_thread_.joinable()) {
//       worker_thread_.join();
//     }
//   }

//   void init()
//   {
//     RCLCPP_INFO(this->get_logger(), "Starting worker thread...");
//     worker_thread_ = std::thread(&UserVisualizationNode::worker_function, this);

//     // Set up a timer to periodically plot the graph
//     timer_ = this->create_wall_timer(
//       std::chrono::seconds(1),
//       std::bind(&UserVisualizationNode::plot_plan, this)
//     );
//   }

// private:
//   void plan_callback(const plansys2_msgs::msg::Plan::SharedPtr msg)
//   {
//     RCLCPP_INFO(this->get_logger(), "Received new plan with %zu items.", msg->items.size());
//     std::lock_guard<std::mutex> lock(mutex_);
//     plan_ = *msg;
//   }

//   void plot_plan()
//   {
//     RCLCPP_INFO(this->get_logger(), "Plotting plan...");

//     plt::clf();
//     plt::title("Plan Visualization");
//     plt::xlabel("Time");
//     plt::ylabel("Robot");

//     std::map<std::string, int> robot_positions;
//     int robot_pos = 0;

//     {
//       std::lock_guard<std::mutex> lock(mutex_);
//       for (const auto& item : plan_.items) {
//         const auto& action = item.action;
//         double start_time = item.time;
//         double end_time = item.time + item.duration;

//         RCLCPP_INFO(this->get_logger(), "Action: %s, Start Time: %.2f, End Time: %.2f", action.c_str(), start_time, end_time);

//         std::string robot = action.substr(1, action.find(' ') - 1); // Extract robot name

//         if (robot_positions.find(robot) == robot_positions.end()) {
//           robot_positions[robot] = robot_pos++;
//         }

//         int pos = robot_positions[robot];
//         plt::plot({start_time, end_time}, {static_cast<double>(pos), static_cast<double>(pos)}, "b-");
//         plt::text((start_time + end_time) / 2.0, static_cast<double>(pos), action);
//       }
//     }

//     for (const auto& robot_position : robot_positions) {
//       plt::text(-1.0, static_cast<double>(robot_position.second), robot_position.first);
//     }

//     // Adjust limits based on the plan
//     plt::xlim(0.0, static_cast<double>(last_plan_size_) * 5.0); // Assuming a maximum of 5 units per action for the initial view
//     plt::ylim(-1.0, static_cast<double>(robot_positions.size()));

//     // Draw the updated plot
//     plt::draw();
//     plt::pause(0.001);

//     RCLCPP_INFO(this->get_logger(), "Plan plotted and updated.");
//   }

//   void worker_function()
//   {
//     while (!stop_requested_.load()) {
//       std::this_thread::sleep_for(std::chrono::seconds(1));
//     }
//   }

//   static void signal_handler(int signal)
//   {
//     if (signal == SIGINT) {
//       rclcpp::shutdown();
//     }
//   }

//   rclcpp::Subscription<plansys2_msgs::msg::Plan>::SharedPtr plan_subscription_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   std::mutex mutex_;
//   plansys2_msgs::msg::Plan plan_;
//   std::thread worker_thread_;
//   std::atomic<bool> stop_requested_;
//   size_t last_plan_size_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<UserVisualizationNode>();

//   node->init();
//   rclcpp::spin(node);
//   rclcpp::shutdown();

//   return 0;
// }


// #include "rclcpp/rclcpp.hpp"
// #include "plansys2_msgs/msg/plan.hpp"
// #include "matplotlibcpp.h"
// #include <memory>
// #include <vector>
// #include <string>
// #include <map>
// #include <mutex>
// #include <thread>
// #include <atomic>
// #include <chrono>
// #include <algorithm>

// namespace plt = matplotlibcpp;

// class UserVisualizationNode : public rclcpp::Node
// {
// public:
//   UserVisualizationNode()
//   : Node("uservisualization_node"), stop_requested_(false), last_plan_size_(0)
//   {
//     RCLCPP_INFO(this->get_logger(), "Initializing UserVisualizationNode...");

//     // Subscriber for the plan
//     plan_subscription_ = this->create_subscription<plansys2_msgs::msg::Plan>(
//       "/executing_plan", 10, std::bind(&UserVisualizationNode::plan_callback, this, std::placeholders::_1));

//     // Setup signal handling for proper shutdown
//     signal(SIGINT, signal_handler);

//     RCLCPP_INFO(this->get_logger(), "UserVisualizationNode initialized.");
//   }

//   ~UserVisualizationNode()
//   {
//     stop_requested_.store(true);
//     if (worker_thread_.joinable()) {
//       worker_thread_.join();
//     }
//   }

//   void init()
//   {
//     RCLCPP_INFO(this->get_logger(), "Starting worker thread...");
//     worker_thread_ = std::thread(&UserVisualizationNode::worker_function, this);

//     // Set up a timer to periodically plot the graph
//     timer_ = this->create_wall_timer(
//       std::chrono::seconds(1),
//       std::bind(&UserVisualizationNode::plot_plan, this)
//     );
//   }

// private:
//   void plan_callback(const plansys2_msgs::msg::Plan::SharedPtr msg)
//   {
//     RCLCPP_INFO(this->get_logger(), "Received new plan with %zu items.", msg->items.size());
//     std::lock_guard<std::mutex> lock(mutex_);
//     plan_ = *msg;
//     last_plan_size_ = msg->items.size();
//   }

//   void plot_plan()
//   {
//     RCLCPP_INFO(this->get_logger(), "Plotting plan...");

//     plt::clf();
//     plt::title("Plan Visualization");
//     plt::xlabel("Time");
//     plt::ylabel("Robot");

//     std::map<std::string, int> robot_positions;
//     int robot_pos = 0;

//     {
//       std::lock_guard<std::mutex> lock(mutex_);
//       for (const auto& item : plan_.items) {
//         const auto& action = item.action;
//         double start_time = item.time;
//         double end_time = item.time + item.duration;

//         RCLCPP_INFO(this->get_logger(), "Action: %s, Start Time: %.2f, End Time: %.2f", action.c_str(), start_time, end_time);

//         std::string robot = action.substr(1, action.find(' ') - 1); // Extract robot name

//         if (robot_positions.find(robot) == robot_positions.end()) {
//           robot_positions[robot] = robot_pos++;
//         }

//         int pos = robot_positions[robot];
//         plt::plot({start_time, end_time}, {static_cast<double>(pos), static_cast<double>(pos)}, "b-");
//         plt::text((start_time + end_time) / 2.0, static_cast<double>(pos), action);
//       }
//     }

//     for (const auto& robot_position : robot_positions) {
//       plt::text(-1.0, static_cast<double>(robot_position.second), robot_position.first);
//     }

//     // Adjust limits based on the plan
//     plt::xlim(0.0, static_cast<double>(last_plan_size_) * 5.0); // Assuming a maximum of 5 units per action for the initial view
//     plt::ylim(-1.0, static_cast<double>(robot_positions.size()));

//     // Draw the updated plot
//     plt::draw();
//     plt::pause(0.001);
//     system("xdotool search --name 'Figure 1' windowmove 1920 0");
//     RCLCPP_INFO(this->get_logger(), "Plan plotted and updated.");
//   }

//   void worker_function()
//   {
//     while (!stop_requested_.load()) {
//       std::this_thread::sleep_for(std::chrono::seconds(1));
//     }
//   }

//   static void signal_handler(int signal)
//   {
//     if (signal == SIGINT) {
//       rclcpp::shutdown();
//     }
//   }

//   rclcpp::Subscription<plansys2_msgs::msg::Plan>::SharedPtr plan_subscription_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   std::mutex mutex_;
//   plansys2_msgs::msg::Plan plan_;
//   std::thread worker_thread_;
//   std::atomic<bool> stop_requested_;
//   size_t last_plan_size_;
// };//   rclcpp::init(argc, argv);


// int main(int argc, char ** argv)
// {
//   auto node = std::make_shared<UserVisualizationNode>();

//   node->init();
//   rclcpp::spin(node);
//   rclcpp::shutdown();

//   return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "matplotlibcpp.h"
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <algorithm>

namespace plt = matplotlibcpp;

class UserVisualizationNode : public rclcpp::Node
{
public:
  UserVisualizationNode()
  : Node("uservisualization_node"), stop_requested_(false), last_plan_size_(0)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing UserVisualizationNode...");

    // Subscriber for the plan
    plan_subscription_ = this->create_subscription<plansys2_msgs::msg::Plan>(
      "/executing_plan", 10, std::bind(&UserVisualizationNode::plan_callback, this, std::placeholders::_1));

    // Setup signal handling for proper shutdown
    signal(SIGINT, signal_handler);
    RCLCPP_INFO(this->get_logger(), "UserVisualizationNode initialized.");
  }

  ~UserVisualizationNode()
  {
    stop_requested_.store(true);
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

  void init()
  {
    RCLCPP_INFO(this->get_logger(), "Starting worker thread...");
    worker_thread_ = std::thread(&UserVisualizationNode::worker_function, this);

    // Set up a timer to periodically plot the graph
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&UserVisualizationNode::plot_plan, this)
    );
  }

private:
  void plan_callback(const plansys2_msgs::msg::Plan::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received new plan with %zu items.", msg->items.size());
    std::lock_guard<std::mutex> lock(mutex_);
    plan_ = *msg;
    last_plan_size_ = msg->items.size();
  }

  void plot_plan()
  {
    // RCLCPP_INFO(this->get_logger(), "Plotting plan...");

    plt::clf();
    plt::title("Plan Visualization");
    plt::xlabel("Time");
    plt::ylabel("Robot");

    std::map<std::string, int> robot_positions = {
        {"robot1", 1}, {"robot2", 2}
    };
    std::map<std::string, std::string> robot_colors = {
        {"robot1", "r-"}, {"robot2", "g-"}
    };
    
    double max_time = 0.0;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto& item : plan_.items) {
        const auto& action = item.action;
        double start_time = item.time;
        double end_time = item.time + item.duration;
        max_time = std::max(max_time, end_time);

        // Extract robot name
        std::string action_str = action;
        size_t start_pos = action_str.find("robot");
        size_t end_pos = action_str.find(' ', start_pos);
        std::string robot = action_str.substr(start_pos, end_pos - start_pos);
        // RCLCPP_INFO(this->get_logger(), "Action %s from robot: %s", action_str.c_str(), robot.c_str());

        // Extract action name
        size_t action_end_pos = action_str.find(' ', 1);
        std::string action_name = action_str.substr(1, action_end_pos - 1);

        int pos = robot_positions[robot];
        plt::plot({end_time, end_time}, {static_cast<double>(pos) - 0.025, static_cast<double>(pos) + 0.025}, {{"color", "black"}, {"linewidth", "2"}});
        plt::plot({start_time, end_time}, {static_cast<double>(pos)- 0.025, static_cast<double>(pos)+ 0.025}, robot_colors[robot]);
        plt::text((start_time + end_time) / 2.0, static_cast<double>(pos), action_name);
        
      }
    }

    for (const auto& robot_position : robot_positions) {
      plt::text(-1.0, static_cast<double>(robot_position.second), robot_position.first);
    }

    // Adjust limits based on the plan
    plt::xlim(0.0, max_time + 5.0); // Adding extra space for readability
    plt::ylim(0.0, 5.0);

    // Draw the updated plot
    plt::draw();
    plt::save("plan_visualization.jpg");
    plt::pause(0.001);
    // system("xdotool search --name 'Figurse 1' windowmove 1600 0");
    // RCLCPP_INFO(this->get_logger(), "Plan plotted and updated.");
  }

  void worker_function()
  {
    while (!stop_requested_.load()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  static void signal_handler(int signal)
  {
    if (signal == SIGINT) {
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<plansys2_msgs::msg::Plan>::SharedPtr plan_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  plansys2_msgs::msg::Plan plan_;
  std::thread worker_thread_;
  std::atomic<bool> stop_requested_;
  size_t last_plan_size_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UserVisualizationNode>();

  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
