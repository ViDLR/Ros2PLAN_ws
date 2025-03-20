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
#include "arms_plan_solver/arms_plan_solver.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace plansys2
{
    ARMSPlanSolver::ARMSPlanSolver() {}

    std::optional<std::filesystem::path>
    ARMSPlanSolver::create_folders(const std::string &node_namespace)
    {
        const std::string base_dir = "/tmp/plan_output/";

        // Directories to be created
        const std::vector<std::string> subdirs = {"", "subproblems/"};

        try {
            for (const auto &subdir : subdirs) {
                std::filesystem::path dir_path = base_dir + subdir;
                if (!std::filesystem::exists(dir_path)) {
                    std::filesystem::create_directories(dir_path);
                }
            }
        } catch (const std::filesystem::filesystem_error &err) {
            RCLCPP_ERROR(lc_node_->get_logger(), "Error creating directories: %s", err.what());
            return std::nullopt;
        }

        return std::filesystem::path(base_dir);
    }

    void ARMSPlanSolver::configure(
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

    std::optional<plansys2_msgs::msg::Plan>ARMSPlanSolver::getPlan(
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
        const auto domain_file_path =  "/tmp/domain.pddl";
        const auto problem_file_path = "/tmp/problem.pddl";

        
        std::ofstream domain_out(domain_file_path);
        std::ofstream problem_out(problem_file_path);

        if (!domain_out || !problem_out) {
            RCLCPP_ERROR(lc_node_->get_logger(), "Failed to open domain or problem file for writing.");
            return {};
        }

        domain_out << domain;
        problem_out << problem;
        // Log the domain and problem names and their contents
        RCLCPP_INFO(lc_node_->get_logger(), "Domain file: %s", domain_file_path);
        // RCLCPP_INFO(lc_node_->get_logger(), "Domain content:\n%s", domain.c_str());
        RCLCPP_INFO(lc_node_->get_logger(), "Problem file: %s", problem_file_path);
        // RCLCPP_INFO(lc_node_->get_logger(), "Problem content:\n%s", problem.c_str());
        

        // Define the path to the Python script
        const std::string script_path = ament_index_cpp::get_package_share_directory("arms_plan_solver") + "/scripts/solve_problem.py";

        if (!std::filesystem::exists(script_path)) {
            RCLCPP_ERROR(lc_node_->get_logger(), "Script not found at path: %s", script_path.c_str());
            return {};
        }

        // Execute the Python script
        // const auto plan_file_path = output_dir / "plan_output/plan_result.txt";
        const auto result_file_path = output_dir / "arms_result.txt";
        const std::string command = "python3 " + script_path + " " +
                                    domain_file_path + " " +
                                    problem_file_path + " " +
                                    "2" +
                                    " > " + result_file_path.string();

        
        RCLCPP_INFO(lc_node_->get_logger(), "Executing command: %s", command.c_str());

        const int status = system(command.c_str());
        if (status != 0) {
            RCLCPP_ERROR(lc_node_->get_logger(), "Failed to execute Python script.");
            return {};
        }

        const auto plan_file_path = output_dir / "mergedplan.txt";
        // Parse the result file
        plansys2_msgs::msg::Plan plan;
        std::ifstream plan_file(plan_file_path);
        if (!plan_file.is_open()) {
            RCLCPP_ERROR(lc_node_->get_logger(), "Failed to open plan result file: %s", plan_file_path.string().c_str());
            return {};
        }

        std::string line;
        while (std::getline(plan_file, line)) {
            if (!line.empty() && line.front() != ';') {
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

        if (plan.items.empty()) {
            RCLCPP_ERROR(lc_node_->get_logger(), "No valid solution found in the plan.");
            return {};
        }

        return plan;
    }

    std::vector<plansys2_msgs::msg::Plan> ARMSPlanSolver::getMultiPathPlan(
        const std::string &domain, const std::string &problem,
        const std::string &node_namespace, const rclcpp::Duration solver_timeout,
        const std::string &mode, const std::vector<std::string> &paths)
    {
        if (system(nullptr) == 0) {
            RCLCPP_ERROR(lc_node_->get_logger(), "System shell not available for executing commands.");
            return {};
        }
    
        const auto output_dir_maybe = create_folders(node_namespace);
        if (!output_dir_maybe) {
            RCLCPP_ERROR(lc_node_->get_logger(), "Failed to create output directories.");
            return {};
        }
        const auto &output_dir = output_dir_maybe.value();
    
        // Write domain and problem to temp files
        const auto domain_file_path = "/tmp/domain.pddl";
        const auto problem_file_path = "/tmp/problem.pddl";
    
        std::ofstream domain_out(domain_file_path);
        std::ofstream problem_out(problem_file_path);
        if (!domain_out || !problem_out) {
            RCLCPP_ERROR(lc_node_->get_logger(), "Failed to write domain or problem file.");
            return {};
        }
    
        domain_out << domain;
        problem_out << problem;
        domain_out.close();
        problem_out.close();
    
        if (domain.empty() || problem.empty()) {
            RCLCPP_ERROR(lc_node_->get_logger(), "Domain or Problem file is empty! Cannot execute planner.");
            return {};
        }
    
        // Handle Full vs Partial Replanning
        if (mode == "full") {
            RCLCPP_INFO(lc_node_->get_logger(), "Full planning requested, resetting plan output.");
            std::filesystem::remove_all(output_dir);
            std::filesystem::create_directories(output_dir);
        }
    
        // Build script command
        std::string script_args = mode;
        for (const auto &path : paths) {
            script_args += " " + path;
        }
    
        // Locate planner script
        const std::string script_path = ament_index_cpp::get_package_share_directory("arms_plan_solver") + "/scripts/solve_problem.py";
        if (!std::filesystem::exists(script_path)) {
            RCLCPP_ERROR(lc_node_->get_logger(), "Script not found at path: %s", script_path.c_str());
            return {};
        }
    
        // Run planner script
        const auto result_file_path = output_dir / "arms_result.txt";
        const std::string command = "python3 " + script_path + " " +
                                    domain_file_path + " " +
                                    problem_file_path + " " +
                                    "2 " + script_args + 
                                    " > " + result_file_path.string();
    
        RCLCPP_INFO(lc_node_->get_logger(), "Executing command: %s", command.c_str());
        const int status = system(command.c_str());
        if (status != 0) {
            RCLCPP_ERROR(lc_node_->get_logger(), "Failed to execute Python script.");
            return {};
        }
    
        // **Parse results dynamically**
        std::vector<plansys2_msgs::msg::Plan> plans;
        std::map<int, plansys2_msgs::msg::Plan> ordered_plans;
        std::vector<plansys2_msgs::msg::Plan> sync_plans;
    
        for (const auto &entry : std::filesystem::directory_iterator(output_dir / "subproblems")) {
            std::string filename = entry.path().filename().string();
    
            if (filename.find("_mergedPLAN.txt") != std::string::npos) {
                plansys2_msgs::msg::Plan plan;
                std::ifstream plan_file(entry.path());
    
                if (!plan_file.is_open()) {
                    RCLCPP_ERROR(lc_node_->get_logger(), "Failed to open plan result file: %s", entry.path().c_str());
                    continue;
                }
    
                std::string path_name = filename.substr(5, filename.find("_mergedPLAN.txt") - 5); // Extract path ID
                bool is_sync_path = path_name.find("sync") != std::string::npos;
                int path_index = is_sync_path ? -1 : std::stoi(path_name);
    
                std::string line;
                while (std::getline(plan_file, line)) {
                    if (!line.empty() && line.front() != ';') { // Ignore comments
                        plansys2_msgs::msg::PlanItem item;
    
                        size_t colon_pos = line.find(":");
                        size_t open_paren_pos = line.find("(");
                        size_t close_paren_pos = line.rfind(")"); // Find last closing parenthesis
                        size_t bracket_pos = line.find("[");
    
                        if (colon_pos == std::string::npos || open_paren_pos == std::string::npos ||
                            close_paren_pos == std::string::npos || bracket_pos == std::string::npos) {
                            RCLCPP_ERROR(lc_node_->get_logger(), "Malformed plan line: %s", line.c_str());
                            continue;
                        }
    
                        try {
                            item.time = std::stof(line.substr(0, colon_pos));
                            item.action = line.substr(open_paren_pos, close_paren_pos - open_paren_pos + 1);  // ✅ Extract full action
                            item.duration = std::stof(line.substr(bracket_pos + 1, line.find("]") - bracket_pos - 1));
    
                            plan.items.push_back(item);
                        } catch (const std::exception &e) {
                            RCLCPP_ERROR(lc_node_->get_logger(), "Error parsing line: %s, Exception: %s", line.c_str(), e.what());
                        }
                    }
                }
    
                if (!plan.items.empty()) {
                    if (is_sync_path) {
                        sync_plans.push_back(plan);
                    } else {
                        ordered_plans[path_index] = plan;
                    }
                }
            }
        }
    
        // **Sort Plans by Order**
        for (auto &pair : ordered_plans) {
            plans.push_back(pair.second);
        }
    
        // **Add Sync Plans at the End**
        plans.insert(plans.end(), sync_plans.begin(), sync_plans.end());
    
        if (plans.empty()) {
            RCLCPP_ERROR(lc_node_->get_logger(), "No valid plans found.");
        } else {
            RCLCPP_INFO(lc_node_->get_logger(), "Successfully parsed %lu path plans in order.", plans.size());
        }
    
        return plans;
    }
    

    
// // **Parse results dynamically**
// std::vector<plansys2_msgs::msg::Plan> plans;
// std::vector<std::string> found_paths;

// for (const auto &entry : std::filesystem::directory_iterator(output_dir / "subproblems")) {
//     std::string filename = entry.path().filename().string();

//     if (filename.find("PATH_") != std::string::npos && filename.find("_mergedPLAN.txt") != std::string::npos) {
//         plansys2_msgs::msg::Plan plan;
//         std::ifstream plan_file(entry.path());

//         if (!plan_file.is_open()) {
//             RCLCPP_ERROR(lc_node_->get_logger(), "Failed to open plan result file: %s", entry.path().c_str());
//             continue;
//         }

//         std::string line;
//         while (std::getline(plan_file, line)) {
//             if (!line.empty() && line.front() != ';') {
//                 plansys2_msgs::msg::PlanItem item;
//                 size_t colon_pos = line.find(":");
//                 size_t paren_pos = line.find(")");
//                 size_t bracket_pos = line.find("[");

//                 item.time = std::stof(line.substr(0, colon_pos));
//                 item.action = line.substr(colon_pos + 2, paren_pos - colon_pos - 1);
//                 item.duration = std::stof(line.substr(bracket_pos + 1, line.find("]") - bracket_pos - 1));

//                 plan.items.push_back(item);
//             }
//         }

//         if (!plan.items.empty()) {
//             plans.push_back(plan);
//             found_paths.push_back(filename);
//         }
//     }
// }

// if (plans.empty()) {
//     RCLCPP_ERROR(lc_node_->get_logger(), "No valid plans found.");
// } else {
//     RCLCPP_INFO(lc_node_->get_logger(), "Successfully parsed %lu path plans: [%s]", 
//                 plans.size(), fmt::format("{}", fmt::join(found_paths, ", ")).c_str());
// }

// return plans;
// }

    bool
    ARMSPlanSolver::isDomainValid(
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
PLUGINLIB_EXPORT_CLASS(plansys2::ARMSPlanSolver, plansys2::PlanSolverBase);
