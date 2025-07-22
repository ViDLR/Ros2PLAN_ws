#include <sys/stat.h>
#include <sys/types.h>

#include <filesystem>
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "plansys2_msgs/msg/plan_item.hpp"
#include "optic_plan_solver/optic_plan_solver.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace plansys2
{

OPTICPlanSolver::OPTICPlanSolver()
{
}

std::optional<std::filesystem::path>
OPTICPlanSolver::create_folders(const std::string & node_namespace)
{
  auto output_dir = lc_node_->get_parameter(output_dir_parameter_name_).value_to_string();

  // Allow usage of the HOME directory with the `~` character, returning if there is an error.
  const char * home_dir = std::getenv("HOME");
  if (output_dir[0] == '~' && home_dir) {
    output_dir.replace(0, 1, home_dir);
  } else if (!home_dir) {
    RCLCPP_ERROR(
      lc_node_->get_logger(), "Invalid use of the ~ character in the path: %s", output_dir.c_str()
    );
    return std::nullopt;
  }

  // Create the necessary folders, returning if there is an error.
  auto output_path = std::filesystem::path(output_dir);
  if (node_namespace != "") {
    for (auto p : std::filesystem::path(node_namespace)) {
      if (p != std::filesystem::current_path().root_directory()) {
        output_path /= p;
      }
    }
    try {
      std::filesystem::create_directories(output_path);
    } catch (std::filesystem::filesystem_error & err) {
      RCLCPP_ERROR(lc_node_->get_logger(), "Error writing directories: %s", err.what());
      return std::nullopt;
    }
  }
  return output_path;
}

void OPTICPlanSolver::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
  const std::string & plugin_name)
{
  lc_node_ = lc_node;

  arguments_parameter_name_ = plugin_name + ".arguments";
  lc_node_->declare_parameter<std::string>(arguments_parameter_name_, "");

  output_dir_parameter_name_ = plugin_name + ".output_dir";
  lc_node_->declare_parameter<std::string>(
    output_dir_parameter_name_, std::filesystem::temp_directory_path());
}

std::optional<plansys2_msgs::msg::Plan>
OPTICPlanSolver::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace,
  const rclcpp::Duration solver_timeout)
{
  if (system(nullptr) == 0) {
    return {};
  }

  // Set up the folders
  const auto output_dir_maybe = create_folders(node_namespace);
  if (!output_dir_maybe) {
    return {};
  }
  const auto & output_dir = output_dir_maybe.value();
  RCLCPP_INFO(
    lc_node_->get_logger(), "Writing planning results to %s.", output_dir.string().c_str());

  // Perform planning
  plansys2_msgs::msg::Plan ret;

  const auto domain_file_path = output_dir / std::filesystem::path("domain.pddl");
  std::ofstream domain_out(domain_file_path);
  domain_out << domain;
  domain_out.close();

  const auto problem_file_path = output_dir / std::filesystem::path("problem.pddl");
  std::ofstream problem_out(problem_file_path);
  problem_out << problem;
  problem_out.close();

  RCLCPP_INFO(
    lc_node_->get_logger(), "[%s-optic-cplex] called with timeout %f seconds",
    lc_node_->get_name(), solver_timeout.seconds());

  std::string solver_path = ament_index_cpp::get_package_share_directory("optic_plan_solver") + "/lib/optic_plan_solver/external_solver/optic-cplex";

  const auto plan_file_path = output_dir / std::filesystem::path("plan");
  // const auto args = lc_node_->get_parameter(arguments_parameter_name_).value_to_string();
  const int status = system(
    (solver_path + " -N " +
    domain_file_path.string() + " " + problem_file_path.string() + " > " + plan_file_path.string())
    .c_str());

  if (status == -1) {
    return {};
  }

  std::string line;
  std::ifstream plan_file(plan_file_path);
  bool solution = false;

  if (plan_file.is_open()) {
    while (getline(plan_file, line)) {
      if (!solution) {
        if (line.find("; Solution Found") != std::string::npos) {
          solution = true;
        }
      } else if (line.front() != ';') {
        plansys2_msgs::msg::PlanItem item;
        size_t colon_pos = line.find(":");
        size_t colon_par = line.find(")");
        size_t colon_bra = line.find("[");

        std::string time = line.substr(0, colon_pos);
        std::string action = line.substr(colon_pos + 2, colon_par - colon_pos - 1);
        std::string duration = line.substr(colon_bra + 1);
        duration.pop_back();

        item.time = std::stof(time);
        item.action = action;
        item.duration = std::stof(duration);

        ret.items.push_back(item);
      }
    }
    plan_file.close();
  }

  if (ret.items.empty()) {
    return {};
  }

  return ret;
}

std::map<std::string, plansys2_msgs::msg::Plan> OPTICPlanSolver::getMultiPathPlan(
    const std::string &domain, 
    const std::string &problem,
    const std::string &node_namespace,
    const rclcpp::Duration solver_timeout,
    const std::string &validation_report_path)
{
    std::map<std::string, plansys2_msgs::msg::Plan> multi_path_plans;

    if (system(nullptr) == 0) {
        return {};
    }

    const auto output_dir_maybe = create_folders(node_namespace);
    if (!output_dir_maybe) {
        return {};
    }

    const auto &output_dir = output_dir_maybe.value();
    const auto domain_file_path = output_dir / "domain.pddl";
    std::ofstream domain_out(domain_file_path);
    domain_out << domain;
    domain_out.close();

    // === Determine paths to solve
    std::vector<std::string> paths;

    if (!validation_report_path.empty() && std::filesystem::exists(validation_report_path)) {
        std::ifstream report_file(validation_report_path);
        nlohmann::json report;
        report_file >> report;
        for (const auto &entry : report["affected_paths"]) {
            paths.push_back(entry.get<std::string>());
        }
    } else {
        paths = {"path0"};  // fallback dummy path
    }

    std::string solver_path = ament_index_cpp::get_package_share_directory("optic_plan_solver") +
                              "/lib/optic_plan_solver/external_solver/optic-cplex";

    for (const auto &path : paths) {
        const auto problem_file_path = output_dir / ("problem_" + path + ".pddl");
        std::ofstream problem_out(problem_file_path);
        problem_out << problem;
        problem_out.close();

        const auto plan_file_path = output_dir / ("plan_" + path);
        int status = system(
            (solver_path + " -N " +
             domain_file_path.string() + " " +
             problem_file_path.string() + " > " +
             plan_file_path.string()).c_str());

        if (status == -1) continue;

        std::ifstream plan_file(plan_file_path);
        if (!plan_file.is_open()) continue;

        plansys2_msgs::msg::Plan plan;
        std::string line;
        bool solution_found = false;

        while (getline(plan_file, line)) {
            if (!solution_found) {
                if (line.find("; Solution Found") != std::string::npos) {
                    solution_found = true;
                }
            } else if (!line.empty() && line.front() != ';') {
                plansys2_msgs::msg::PlanItem item;

                size_t colon_pos = line.find(":");
                size_t close_paren = line.find(")");
                size_t bracket_open = line.find("[");

                if (colon_pos == std::string::npos || close_paren == std::string::npos || bracket_open == std::string::npos)
                    continue;

                try {
                    item.time = std::stof(line.substr(0, colon_pos));
                    item.action = line.substr(colon_pos + 2, close_paren - colon_pos - 1);
                    std::string duration = line.substr(bracket_open + 1);
                    duration.pop_back();
                    item.duration = std::stof(duration);
                    plan.items.push_back(item);
                } catch (...) {
                    continue;
                }
            }
        }

        if (!plan.items.empty()) {
            multi_path_plans[path] = plan;
        }
    }

    return multi_path_plans;
}



bool
OPTICPlanSolver::isDomainValid(
  const std::string & domain,
  const std::string & node_namespace)
{
  if (system(nullptr) == 0) {
    return false;
  }

  // Set up the folders
  const auto output_dir_maybe = create_folders(node_namespace);
  if (!output_dir_maybe) {
    return {};
  }
  const auto & output_dir = output_dir_maybe.value();
  RCLCPP_INFO(
    lc_node_->get_logger(), "Writing domain validation results to %s.",
    output_dir.string().c_str()
  );

  // Perform domain validation
  const auto domain_file_path = output_dir / std::filesystem::path("check_domain.pddl");
  std::ofstream domain_out(domain_file_path);
  domain_out << domain;
  domain_out.close();

  const auto problem_file_path = output_dir / std::filesystem::path("check_problem.pddl");
  std::ofstream problem_out(problem_file_path);
  problem_out << "(define (problem void) (:domain plansys2))";
  problem_out.close();
  std::string solver_path = ament_index_cpp::get_package_share_directory("optic_plan_solver") + "/lib/optic_plan_solver/external_solver/optic-cplex";

  const auto plan_file_path = output_dir / std::filesystem::path("check.out");
  const int status = system(
    (solver_path + " -N " + domain_file_path.string() + " " + problem_file_path.string() + " > " + plan_file_path.string())
    .c_str());

  if (status == -1) {
    return false;
  }

  std::string line;
  std::ifstream plan_file(plan_file_path);
  bool solution = false;

  if (plan_file && plan_file.is_open()) {
    while (getline(plan_file, line)) {
      if (!solution) {
        if (line.find("Solution Found") != std::string::npos) {
          solution = true;
        }
      }
    }
    plan_file.close();
  }

  return solution;
}

}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plansys2::OPTICPlanSolver, plansys2::PlanSolverBase);

