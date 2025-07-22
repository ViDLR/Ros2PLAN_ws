import sys
from itertools import combinations
import os
import subprocess
import time
import scipy.stats as scistat
import shutil
from collections import defaultdict
from pathlib import Path
from ament_index_python.packages import get_package_prefix
import matplotlib.pyplot as plt
import networkx as nx
import matplotlib.patches as mpatches
import json
import tempfile

# Dynamically add the directory to the Python path
# Define the output paths dynamically
base_output_dir = Path("/tmp/plan_output/")
base_output_dir_str ="/tmp/plan_output/"
validation_output_dir = "/tmp/validation/"
subproblems_dir = base_output_dir / "subproblems"

# Ensure the directories exist
subproblems_dir.mkdir(parents=True, exist_ok=True)

script_dir = os.path.dirname(__file__)  # Get the directory where the script is located
parent_dir = os.path.dirname(script_dir)  # Navigate to the parent directory if needed
external_solver_dir = os.path.join(get_package_prefix("arms_plan_solver"), "lib", "arms_plan_solver", "external_solver")
optic_cplex_path = os.path.join(external_solver_dir, "optic-cplex")
sys.path.append(parent_dir)  # Add the parent directory to sys.path
sys.path.append(script_dir + "/functions")

import arms_utils.Class_and_functions as Class_and_functions  # Import ARMS Toolbox functions  # Import ARMS Toolbox functions

#  Parameters for planning
domain_file = sys.argv[1]
problem_file = sys.argv[2]
min_nb_cluster = int(sys.argv[3])
validation_report_path = sys.argv[4] if len(sys.argv) > 4 else "" 

# print(validation_report_path)

# (:requirements :strips :typing :fluents :negative-preconditions :timed-initial-literals :disjunctive-preconditions :durative-actions :universal-preconditions )

def export_validation_files(path_idx, pathplan, subproblem_paths):
    """
    Save all PDDL and plan files for validation, and write pathway.txt with the correct execution order.
    """

    # print(f"📁 Exporting validation files for PATH_{path_idx}")
    # print(f"    Pathplan files: {pathplan}")


    validation_path = Path(f"/tmp/validation/subproblems/PATH_{path_idx}")
    validation_path.mkdir(parents=True, exist_ok=True)

    pathway_file = validation_path / "pathway.txt"

    with open(pathway_file, "w") as pf:
        for i, plan_file in enumerate(pathplan):
            base = plan_file.replace("_PLAN.txt", "")
            pf.write(f"{os.path.basename(base)}\n")

            for suffix in [".pddl", "_PLAN.txt"]:
                src = f"{base}{suffix}"
                dst = validation_path / os.path.basename(src)
                try:
                    shutil.copy(src, dst)
                except FileNotFoundError:
                    print(f"❌ Missing file for validation: {src}")


domain_file = Class_and_functions.check_domain_arms(domain_file)
output_file = base_output_dir / "arms_output.txt"

# For subproblems:
def main():
    """ 
    - Assigns robots to paths based on site clusters
    - Ensures robots in the same locations are properly grouped
    - Generates and solves planning problems
    - Merges plans correctly 
    """

    # Load mission
    mission = Class_and_functions.recover_mission_from_json("/tmp/world_info.json")

    robot_states = {
    r.name: {"site": "base", "position": "cpbase", "conf": "groundconf", "loc": mission.sites[0].center, "time": 0}
    for r in mission.robots
    }  # Store last known state of each robot globally

    executed_paths = []
    new_unique_paths = []
    sequential_links = {}

    if validation_report_path and os.path.exists(validation_report_path):
        print(f"📄 Using validation report: {validation_report_path}")
        with open(validation_report_path) as f:
            report = json.load(f)

        # Extract relevant report fields
        best_alloc = report["best_alloc"]
        arms_paths = report["arms_paths"]
        parsed_plan = report.get("parsed_plan", [])
        path_viability = report.get("path_viability", {})
        fixed_paths = [str(pid) for pid in report.get("fixed_paths", [])]
        sequential_links = {
            str(k): [str(x) for x in v]
            for k, v in report.get("sequential_links", {}).items()
        }

        # Determine executed paths (fixed + unaffected)
        unaffected_paths = [
            pid for pid, data in path_viability.items()
            if data.get("status") == "unaffected" and data.get("valid")
        ]
        executed_paths = list(set(fixed_paths + unaffected_paths))
        print(f"🟢 Executed paths (fixed + unaffected): {executed_paths}")

        # Build unique path structures
        new_unique_paths = [{
            "id": str(p["id"]),
            "path": p["path"],
            "robots": p["robots"]
        } for p in arms_paths]

        print("\nRecovered unique paths from report:")
        for p in new_unique_paths:
            print(f"Path ID: {p['id']} | Sites: {p['path']} | Robots: {p['robots']}")

        # Identify robots involved in fixed paths
        fixed_robots = set()
        for p in new_unique_paths:
            if p["id"] in fixed_paths:
                fixed_robots.update(p["robots"])

        # Extract all robots mentioned in the parsed plan
        robots_in_plan = {robot for a in parsed_plan for robot in a["robots"]}

        # Initialize robot_state for all robots in the plan
        robot_state = {
            r: {
                "position": None,
                "conf": None,
                "time": 0.0,
                "site": None,
                "loc": None
            } for r in robots_in_plan
        }

        # Dump parsed_plan into a temp file to reuse p_plan()
        with tempfile.NamedTemporaryFile(mode='w+', delete=False) as tmp_plan_file:
            for a in parsed_plan:
                action_line = f"{a['start_time']:.3f}: {a['full_action']} [{a['duration']:.3f}]\n"
                tmp_plan_file.write(action_line)
            tmp_plan_path = tmp_plan_file.name

        # Call p_plan to update robot_state from the parsed plan
        robot_state, max_time, robot_exec_times = Class_and_functions.p_plan(
            tmp_plan_path, robot_state, mission
        )

        # Filter to keep only states of robots from fixed paths
        robot_state = {r: s for r, s in robot_state.items() if r in fixed_robots}

        print("\n🗺️ Reconstructed robot states after fixed paths (via p_plan):")
        for r, s in robot_state.items():
            print(f"{r}: pos={s['position']} | conf={s['conf']} | site={s['site']} | loc={s['loc']} | t={s['time']:.1f}")

        stn = Class_and_functions.build_STN(
            new_unique_paths,
            sequential_links,
            executed_paths=executed_paths
        )

    else:
        print("🆕 No validation report provided → running full mission planning")
        
        clusters, allocationscenario = Class_and_functions.getbestassignfrommission(mission=mission, minnbofcluster=min_nb_cluster)

        print("Clusters:")
        for c in clusters:
            print(c.name)
            for s in c.sites:
                print(s.name)
            print("\n")

        # Assign robots to unique path groups
        robotpath = defaultdict(list)
        print("Robot assignment:")
        for r in mission.robots:
            print(r.name, "assigned to", r.histo)
            path_tuple = tuple(r.histo)
            robotpath[path_tuple].append(r.name)

        # Generate unique paths and extract links
        unique_paths_with_robots = [{'path': list(path), 'robots': robots} for path, robots in robotpath.items()]
        new_unique_paths = Class_and_functions.split_redundant_paths(unique_paths_with_robots, mission)
        sequential_links = Class_and_functions.find_sequential_links(new_unique_paths, mission)

        stn = Class_and_functions.build_STN(
            new_unique_paths,
            sequential_links,
        )

    
    # DRAWING OF GRAPH FOR THE ARMS OUPUT
    # allocations, site_positions, color_map = Class_and_functions.extract_inputs_from_mission(mission)

    # site_color_map = {
    #     "base": "red", "4": "steelblue", "7": "forestgreen", "2": "purple",
    #     "6": "yellow", "1": "saddlebrown", "3": "hotpink", "5": "lightgray"
    # }

    # # === Ensure keys are strings for plotting ===
    # allocations_str = {r: [str(s) for s in seq] for r, seq in allocations.items()}
    # site_positions_str = {str(k): v for k, v in site_positions.items()}

    # robassign={
    #     "robot0": ["base", "3", "5"],
    #     "robot1": ["base", "3", "5"],
    #     "robot2": ["base", "1", "6"],
    #     "robot3": ["base", "1", "6"],
    #     "robot4": ["base", "1", "2"],
    #     "robot5": ["base", "4", "7", "2"],
    #     "robot6": ["base", "4", "7", "2"],
    # }

    # print(type(robassign)) 

    # # === Call Plot 1: Topological Allocation ===
    # Class_and_functions.plot_topological_multigraph(robassign, mission.sites, color_map, site_color_map)

    # # === Define cluster_paths manually or from ARMS ===
    # cluster_paths = {
    #     "Team_0": {"cluster": 1, "robots": ["robot0", "robot1"], "sites": ["base", "3", "5"]},
    #     "Team_1": {"cluster": 0, "robots": ["robot2", "robot3"], "sites": ["1", "6"]},
    #     "Team_2": {"cluster": 0, "robots": ["robot2", "robot3", "robot4"], "sites": ["base", "1"]},
    #     "Team_3": {"cluster": 0, "robots": ["robot4"], "sites": ["1", "2"]},
    #     "Team_4": {"cluster": 0, "robots": ["robot5", "robot6"], "sites": ["base", "4", "7", "2"]},
    # }

    # team_colors = {"Team_0": "navy", "Team_1": "royalblue", "Team_2": "darkblue", "Team_3": "purple", "Team_4": "pink"}
    # linestyles = {"Team_0": "dashed", "Team_1": "dashdot", "Team_2": "dotted", "Team_3": "solid", "Team_4": "dotted"}

    # # === Call Plot 2: Cluster Paths ===
    # Class_and_functions.plot_cluster_paths(cluster_paths, site_positions_str, site_color_map, team_colors, linestyles)

    # # === Define visit_data manually or from plans ===
    # visit_data = {
    #     "robot0": [("base", 0, 10), ("3", 295, 512), ("5", 922, 1197)],
    #     "robot1": [("base", 0, 10), ("3", 283, 542), ("5", 937, 1220)],
    #     "robot2": [("base", 0, 10), ("1", 352, 473), ("6", 775, 856)],
    #     "robot3": [("base", 0, 10), ("1", 352, 473), ("6", 777, 858)],
    #     "robot4": [("base", 0, 10), ("1", 352, 473), ("2", 1178, 1268)],
    #     "robot5": [("base", 0, 10), ("4", 306, 388), ("7", 610, 787), ("2", 1178, 1268)],
    #     "robot6": [("base", 0, 10), ("4", 304, 371), ("7", 630, 787), ("2", 1178, 1268)],
    # }

    # waiting_data = {
    #     "robot4": [(1118, 1178)]  # waited 60s before others arrived at site2
    # }

    # # === Call Plot 3: Site Visits Over Time ===
    # Class_and_functions.plot_site_visits_over_time(visit_data, site_color_map, waiting_data)
    # plt.show()

    # Finally: build the STN using all extracted or computed inputs
    

    # mergedplans = []

    # print("THE ROBOT STATES", robot_states, "\n")
    

    Class_and_functions.draw_STN(stn, impact_colors={0: "lightcoral", "sync_0_a": "orange"})

    while not all(stn.nodes[p]["executed"] for p in stn.nodes if p not in ["Start", "End"]):  # Excluding "Start" and "End" nodes
        executable_paths = Class_and_functions.get_executable_paths(stn)

        for path_idx in executable_paths:
            print("Solving path : ", path_idx)
            sync_created = False
            path_info = stn.nodes[path_idx]
            pathplan = []
            path = path_info["sites"]
            robots = path_info["robots"]

            if "End" in stn.successors(path_idx) and mission.sites[0].name not in path:
                path.append(mission.sites[0].name)

            # print(f"\n🚀 Executing path {path_idx}: {path} with robots {robots}")

            # Retrieve last known state for each robot
            robot_state = {robot: robot_states[robot] for robot in robots}
            previous_robotstate = {r: dict(robot_states[r]) for r in robots}

            # **Step 1: Handle Multi-Predecessor Synchronization (Dynamic "Goto" Paths)**
            predecessors = list(stn.predecessors(path_idx))
            site_obj = next((s for s in mission.sites if s.name == path[0]), None)

            # === STEP 1: Synchronization (Multi-predecessor case) loop ===
            if len(predecessors) > 1 and not any(robot_state[robot]["site"] == path[0] for robot in robots):
                travel_time = {}
                finishing_times = {}
                arrival_times = {}

                for p in predecessors:
                    for robot in stn.nodes[p]["robots"]:
                        if robot in robots:  # ✅ Only consider robots assigned to path_idx
                            # travel_time = Class_and_functions.distance(robot_states[robot]["loc"], site_obj.center) / 1.5
                            travel_time[robot] = Class_and_functions.distance(robot_states[robot]["loc"], site_obj.center) / 1.5
                            finishing_times[robot] = stn.nodes[p]["latest_finish"]
                            arrival_times[robot] = stn.nodes[p]["latest_finish"] + travel_time[robot]

                # print(travel_time, finishing_times,arrival_times)            
                # **No Sync Path Scenario**: Comparison of max and min finishing time
                total_idle_no_sync = abs(finishing_times[max(finishing_times)]-finishing_times[min(finishing_times)])

                # Step 2: Initialize Sync Scenario Dictionary
                sync_scenarios = {'no_sync':total_idle_no_sync}  # {scenario_name: total_idle_time}
                
                # Step 2: Compute Sync Scenarios
                for i in range(1, len(predecessors) + 1):  # Consider 1 to all predecessors
                    for combination in combinations(predecessors, i):  # Generate sync scenarios
                        sync_scenario_name = f"sync_{'_'.join(map(str, combination))}"
                        
                        # Adjust arrival times by adding travel time to only those in sync paths
                        new_arrival_times = finishing_times.copy()  # Start with finish times (no travel yet)

                        for p in combination:
                            for robot in stn.nodes[p]["robots"]:
                                if robot in robots:  # ✅ Only assigned robots matter
                                    new_arrival_times[robot] += travel_time[robot]  # Travel only for sync path robots

                        # Compute new idle time (latest arrival - earliest arrival)
                        latest_new_arrival = max(new_arrival_times.values())
                        earliest_new_arrival = min(new_arrival_times.values())

                        total_idle_with_sync = latest_new_arrival - earliest_new_arrival
                        sync_scenarios[sync_scenario_name] = total_idle_with_sync  # Store scenario

                # Step 4: Select the Best Sync Strategy
                best_sync_strategy = min(sync_scenarios.items(), key=lambda x: x[1])[0]  # Get key (not value)  # Minimize idle time
                
                if best_sync_strategy == "no_sync":
                    # print("✅ No sync path needed.")
                    continue  # Proceed without sync

                # Step 2: Extract Which Paths Are in the Sync
                sync_predecessors = best_sync_strategy.split("_")[1:]  # Extract path indices
                sync_predecessors = [int(p) for p in sync_predecessors if p.isdigit()]  # Convert to int

                # print(f"⚠️ Best Sync Strategy: {best_sync_strategy}. Syncing {sync_predecessors}.")

                # Step 3: Group Robots by Predecessor Path
                robots_by_predecessor = {}  # Dictionary to store robots per predecessor
                for p in sync_predecessors:
                    sync_robots = [robot for robot in stn.nodes[p]["robots"] if robot in robots]  # Only assigned robots
                    if sync_robots:
                        robots_by_predecessor[p] = sync_robots

                # Step 4: Create Sync Nodes for These Paths
                for p, sync_robots in robots_by_predecessor.items():
                    sync_path_id = f"sync_{path_idx}_{p}"  # One sync node per predecessor path
                    # print(f"🔹 Creating Sync Path {sync_path_id} for robots {sync_robots} (from {p})")

                    # --- ⏱ Correct Sync Time Interval: Use all robots in this sync group
                    earliest_start = max([finishing_times[r] for r in sync_robots])
                    latest_finish  = max([arrival_times[r] for r in sync_robots])
                    execution_time = round(latest_finish - earliest_start, 2)

                    # Add Sync Node to STN (with corrected duration)
                    stn.add_node(
                        sync_path_id,
                        sites=[path[0]],
                        robots=sync_robots,
                        earliest_start=earliest_start,
                        latest_finish=latest_finish,
                        execution_time=execution_time,
                        executed=False
                    )

                    # Adjust STN edges
                    stn.remove_edge(p, path_idx)  # Remove direct link

                    # Add sync transition: travel only
                    sync_duration = round(latest_finish - earliest_start, 2)
                    travel_only = round(earliest_start - finishing_times[sync_robots[0]], 2)  # Could average if you prefer

                    stn.add_edge(p, sync_path_id, min_time_gap=travel_only)
                    stn.add_edge(sync_path_id, path_idx, min_time_gap=0)  # Sync ends right before next path starts



                sync_created = True  # Mark that we modified the STN
                
            # A sync path has been created so we need to execute it first
            if sync_created:
                break
            

            # **Step 3: Solve Each Step in the Path**
            for i in range(len(path)):
                # print(f"🔹 Solving: {[r for r in robot_state.keys()]} -> {path[i]}")

                # Define goal type
                if path[i] == "base" or ("sync" in str(path_idx) and len(path) == 1):
                    goal_type = "goto"
                else:
                    goal_type = "solving"

                # Generate problem
                # print(robot_state)
                prblm_path = Class_and_functions.GenerateAdaptedProblem(mission, problem_file, robot_state, path[i], goal_type)
                # print(prblm_path)
                # Solve the problem
                try:
                    Planner_command = "timeout -v 1m {2} -N {0} {1}.pddl".format(domain_file, prblm_path, optic_cplex_path)
                    time_output_file = "{0}_time_output.txt".format(prblm_path)
                    PDDL_outputfile = "{0}_PLAN.txt".format(prblm_path)
                    pathplan.append(PDDL_outputfile)

                    # Combine the python command with /usr/bin/time, redirecting /usr/bin/time output to its file
                    command = f"/usr/bin/time -v {Planner_command} > {PDDL_outputfile} 2> {time_output_file}"

                    # Execute the combined command
                    subprocess.run(command, shell=True)
                    Class_and_functions.trim_plan_file("{0}_PLAN.txt".format(prblm_path))
                except IndexError:
                    print("INDEX ERROR")

                previous_exec_time = max(robot_state[r]["time"] for r in robot_state)
                # Update robot state
                robot_state, plan_end_time, _ = Class_and_functions.p_plan(f"{prblm_path}_PLAN.txt", robot_state, mission)

                max_execution_time = plan_end_time +  previous_exec_time
                for r in robot_state:
                    robot_state[r]["time"] =max_execution_time
                # print("max_execution_time",max_execution_time, "\n")

            # print("THE ROBOT STATES", robot_states, "\n")

            merged_path_file = f"{subproblems_dir}/PATH_{path_idx}_mergedPLAN.txt"
            # print(pathplan, merged_path_file, path)
            Class_and_functions.merge_for_single_path(pathplan, merged_path_file, domain_file, Path(problem_file))
            # Save validation files
            # print(f"📦 Saving validation path for PATH_{path_idx}")
            subproblem_paths = [pf.replace("_PLAN.txt", ".pddl") for pf in pathplan]
            export_validation_files(path_idx, pathplan, subproblem_paths)
            # mergedplans.append(Path(merged_path_file))

            # Update STN with execution times
            Class_and_functions.update_STN_temporal_links(stn, path_idx, max_execution_time)

            

            # Count total plan length (aggregated across all problem steps)
            plan_lines=0
            if os.path.isfile(merged_path_file):
                with open(merged_path_file, "r") as f:
                    plan_lines = sum(1 for line in f if line.strip() and not line.strip().startswith(";"))

            # print(plan_lines)

            synced_too_long = False
            if (
                plan_lines > 40
                and len(path) > 1
                and "sync" not in str(path_idx)
                and not any("sync" in s for s in stn.successors(path_idx))
            ):
                split_index = len(path) // 2
                path1 = path[:split_index]
                path2 = path[split_index:]

                stn.nodes[path_idx]["sites"] = path1

                sync_path_id = f"sync_{path_idx}_a"
                stn.add_node(
                    sync_path_id,
                    sites=path2,
                    robots=robots,
                    earliest_start=0,
                    latest_finish=float("inf"),
                    execution_time=0,
                    executed=False
                )

                for succ in list(stn.successors(path_idx)):
                    edge_data = stn.get_edge_data(path_idx, succ)
                    stn.remove_edge(path_idx, succ)
                    stn.add_edge(sync_path_id, succ, **edge_data)

                stn.add_edge(path_idx, sync_path_id, min_time_gap=stn.nodes[path_idx]["execution_time"])

                print(f"📎 Splitting after execution: Path {path_idx} had {plan_lines} actions")
                print(f"    🔹 {path1} (ID: {path_idx})")
                print(f"    🔹 {path2} (ID: {sync_path_id})")

                # ✅ Rewind robot state based on path1 execution
                print("🔁 Recomputing robot state for split path (only path1 executed)")

                temp_robot_state = {r: dict(robot_states[r]) for r in robots}
                split_robot_state = {r: dict(temp_robot_state[r]) for r in temp_robot_state}

                for i in range(len(path1)):
                    goal_type = "goto" if path1[i] == "base" else "solving"
                    temp_prblm_path = Class_and_functions.GenerateAdaptedProblem(mission, problem_file, split_robot_state, path1[i], goal_type)
                    try:
                        Planner_command = "timeout -v 1m {2} -N {0} {1}.pddl".format(domain_file, temp_prblm_path, optic_cplex_path)
                        time_output_file = f"{temp_prblm_path}_time_output.txt"
                        PDDL_outputfile = f"{temp_prblm_path}_PLAN.txt"
                        command = f"/usr/bin/time -v {Planner_command} > {PDDL_outputfile} 2> {time_output_file}"
                        subprocess.run(command, shell=True)
                        Class_and_functions.trim_plan_file(PDDL_outputfile)
                    except IndexError:
                        print("INDEX ERROR during state rewind")

                    prev_time = max(split_robot_state[r]["time"] for r in split_robot_state)
                    split_robot_state, plan_end_time, _ = Class_and_functions.p_plan(PDDL_outputfile, split_robot_state, mission)
                    max_exec = plan_end_time + prev_time
                    for r in split_robot_state:
                        split_robot_state[r]["time"] = max_exec

                # ✅ Replace robot state
                for r in robots:
                    robot_states[r] = previous_robotstate[r]

                synced_too_long = True

            if not synced_too_long:
                # **Mark the Node as Executed Instead of Removing It**
                for robot in robots:
                    robot_states[robot] = robot_state[robot]  # Save robot's latest known state
                stn.nodes[path_idx]["executed"] = True

    Class_and_functions.draw_STN(stn, impact_colors={3:"orange",0: "lightcoral", "sync_0_a": "orange"})
    t_alloc_full_1 = time.perf_counter()
    Class_and_functions.extract_paths_with_dependencies(stn)
    # print(t_alloc_full_1-t_alloc_full_0)
if __name__ == '__main__':
    main()
