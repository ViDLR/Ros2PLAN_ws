// Copyright 2024 Virgile de La Rochefoucauld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <sys/stat.h>
#include <sys/types.h>

#include <filesystem>
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "plansys2_msgs/msg/plan_item.hpp"
#include "iros_plan_solver/iros_plan_solver.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace plansys2
{
    IROSPlanSolver::IROSPlanSolver() {}

    std::optional<std::filesystem::path>
    IROSPlanSolver::create_folders(const std::string &node_namespace)
    {
        auto output_dir = lc_node_->get_parameter(output_dir_parameter_name_).value_to_string();

        // Handle home directory expansion
        const char *home_dir = std::getenv("HOME");
        if (output_dir[0] == '~' && home_dir)
        {
            output_dir.replace(0, 1, home_dir);
        }
        else if (!home_dir)
        {
            RCLCPP_ERROR(lc_node_->get_logger(), "Invalid use of the ~ character in the path: %s", output_dir.c_str());
            return std::nullopt;
        }

        auto output_path = std::filesystem::path(output_dir);
        if (node_namespace != "")
        {
            for (auto p : std::filesystem::path(node_namespace))
            {
                if (p != std::filesystem::current_path().root_directory())
                {
                    output_path /= p;
                }
            }
            try
            {
                std::filesystem::create_directories(output_path);
            }
            catch (std::filesystem::filesystem_error &err)
            {
                RCLCPP_ERROR(lc_node_->get_logger(), "Error creating directories: %s", err.what());
                return std::nullopt;
            }
        }
        return output_path;
    }

    void IROSPlanSolver::configure(
        rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
        const std::string &plugin_name)
    {
        lc_node_ = lc_node;

        arguments_parameter_name_ = plugin_name + ".arguments";
        lc_node_->declare_parameter<std::string>(arguments_parameter_name_, "");

        output_dir_parameter_name_ = plugin_name + ".output_dir";
        lc_node_->declare_parameter<std::string>(
            output_dir_parameter_name_, std::filesystem::temp_directory_path());
    }

    std::optional<plansys2_msgs::msg::Plan>IROSPlanSolver::getPlan(
        const std::string &domain, const std::string &problem,
        const std::string &node_namespace,
        const rclcpp::Duration solver_timeout)
    {
        if (system(nullptr) == 0) {
            RCLCPP_ERROR(lc_node_->get_logger(), "System shell not available for executing commands.");
            return {};
        }

        // Create output directories for temporary files
        const auto output_dir_maybe = create_folders(node_namespace);
        if (!output_dir_maybe) {
            RCLCPP_ERROR(lc_node_->get_logger(), "Failed to create output directories.");
            return {};
        }
        const auto &output_dir = output_dir_maybe.value();
        RCLCPP_INFO(lc_node_->get_logger(), "Output directory: %s", output_dir.string().c_str());

        // Write domain and problem files
        const auto domain_file_path = output_dir / "domain.pddl";
        const auto problem_file_path = output_dir / "problem.pddl";

        
        std::ofstream domain_out(domain_file_path);
        std::ofstream problem_out(problem_file_path);

        if (!domain_out || !problem_out) {
            RCLCPP_ERROR(lc_node_->get_logger(), "Failed to open domain or problem file for writing.");
            return {};
        }

        domain_out << domain;
        problem_out << problem;
        // Log the domain and problem names and their contents
        RCLCPP_INFO(lc_node_->get_logger(), "Domain file: %s", domain_file_path.string().c_str());
        // RCLCPP_INFO(lc_node_->get_logger(), "Domain content:\n%s", domain.c_str());
        RCLCPP_INFO(lc_node_->get_logger(), "Problem file: %s", problem_file_path.string().c_str());
        // RCLCPP_INFO(lc_node_->get_logger(), "Problem content:\n%s", problem.c_str());
        

        // Define the path to the Python script
        const std::string script_path = ament_index_cpp::get_package_share_directory("iros_plan_solver") + "/scripts/solve_problem.py";

        if (!std::filesystem::exists(script_path)) {
            RCLCPP_ERROR(lc_node_->get_logger(), "Script not found at path: %s", script_path.c_str());
            return {};
        }

        // Execute the Python script
        const auto plan_file_path = output_dir / "plan_result.txt";
        const std::string command = "python3 " + script_path + " " +
                                    domain_file_path.string() + " " +
                                    problem_file_path.string() + " " +
                                    "2" +
                                    " > " + plan_file_path.string();

        RCLCPP_INFO(lc_node_->get_logger(), "Executing command: %s", command.c_str());

        const int status = system(command.c_str());
        if (status != 0) {
            RCLCPP_ERROR(lc_node_->get_logger(), "Failed to execute Python script.");
            return {};
        }

        // Parse the result file
        plansys2_msgs::msg::Plan plan;
        std::ifstream plan_file(plan_file_path);
        if (!plan_file.is_open()) {
            RCLCPP_ERROR(lc_node_->get_logger(), "Failed to open plan result file: %s", plan_file_path.string().c_str());
            return {};
        }

        std::string line;
        bool solution_found = false;
        while (std::getline(plan_file, line)) {
            if (!solution_found) {
                if (line.find("Solution Found") != std::string::npos) {
                    solution_found = true;
                }
            } else if (!line.empty() && line.front() != ';') {
                plansys2_msgs::msg::PlanItem item;

                // Parse plan line: Time: Action [Duration]
                try {
                    size_t colon_pos = line.find(":");
                    size_t paren_pos = line.find(")");
                    size_t bracket_pos = line.find("[");

                    item.time = std::stof(line.substr(0, colon_pos));
                    item.action = line.substr(colon_pos + 2, paren_pos - colon_pos - 1);
                    item.duration = std::stof(line.substr(bracket_pos + 1, line.find("]") - bracket_pos - 1));

                    plan.items.push_back(item);
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(lc_node_->get_logger(), "Error parsing plan item: %s", e.what());
                }
            }
        }

        if (plan.items.empty() || !solution_found) {
            RCLCPP_ERROR(lc_node_->get_logger(), "No valid solution found in the plan.");
            return {};
        }

        return plan;
    }



    bool
    IROSPlanSolver::isDomainValid(
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
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plansys2::IROSPlanSolver, plansys2::PlanSolverBase);
