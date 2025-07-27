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
import copy

def export_networkx_stn(nx_graph):
    nodes = {}
    edges = []
    clusters = {}

    for node, data in nx_graph.nodes(data=True):
        nodes[node] = data
        cluster = data.get("cluster")
        if cluster:
            clusters.setdefault(cluster, []).append(node)

    for src, dst, data in nx_graph.edges(data=True):
        edges.append({
            "src": src,
            "dst": dst,
            "label": data.get("label", "")
        })

    print("🧠 Nodes:")
    for node_id, attr in nodes.items():
        print(f"  {node_id}: {attr}")

    print("\n🔗 Edges:")
    for e in edges:
        print(f"  {e['src']} -> {e['dst']} [label=\"{e['label']}\"]")

    print("\n📦 Clusters:")
    for cluster_name, node_list in clusters.items():
        print(f"  {cluster_name}: {node_list}")

    return {"nodes": nodes, "edges": edges, "clusters": clusters}


#  Parameters for planning
domain_file = sys.argv[1]
problem_file = sys.argv[2]
min_nb_cluster = int(sys.argv[3])
validation_report_path = sys.argv[4] if len(sys.argv) > 4 else "" 
# Optional output override
if len(sys.argv) > 5 and sys.argv[5]:
    base_output_dir = Path(sys.argv[5])
else:
    base_output_dir = Path("/tmp/plan_output/")

base_output_dir_str = str(base_output_dir)

# Define the output paths dynamically
subproblems_dir = base_output_dir / "subproblems"
script_dir = os.path.dirname(__file__)  # Get the directory where the script is located
parent_dir = os.path.dirname(script_dir)  # Navigate to the parent directory if needed
external_solver_dir = os.path.join(get_package_prefix("arms_plan_solver"), "lib", "arms_plan_solver", "external_solver")
optic_cplex_path = os.path.join(external_solver_dir, "optic-cplex")
sys.path.append(parent_dir)  # Add the parent directory to sys.path
sys.path.append(script_dir + "/functions")
# Ensure the directories exist
subproblems_dir.mkdir(parents=True, exist_ok=True)

import arms_utils.Class_and_functions as Class_and_functions  # Import ARMS Toolbox functions  # Import ARMS Toolbox functions

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
    # mission = Class_and_functions.recover_mission_from_json("/tmp/world_info.json")
    mission = Class_and_functions.getdataclassfrompddl(problem_file)

    # robot_states = {
    # r.name: {"site": "base", "position": "cpbase", "conf": "groundconf", "loc": mission.sites[0].center, "time": 0}
    # for r in mission.robots
    # }  # Store last known state of each robot globally

    executed_paths = []
    new_unique_paths = []
    sequential_links = {}

    if validation_report_path and os.path.exists(validation_report_path):
        print(f"📄 Using validation report: {validation_report_path}")
        with open(validation_report_path) as f:
            report = json.load(f)
        if report.get("replan_required", True):
            print("🔁 Replan required according to validation report. Re-running ARMS on failed_problem.pddl")

            # Reconstruct mission state from failed_problem.pddl (ARMS input)
            failed_problem_path = "/tmp/validation/failed_problem.pddl"
            mission = Class_and_functions.getdataclassfrompddl(failed_problem_path)
            robots = copy.deepcopy(mission.robots)

            t_alloc_full_0 = time.perf_counter()

            clusters, allocationscenario = Class_and_functions.getbestassignfrommission(mission=mission, minnbofcluster=1)


            # Build robot-to-path mapping from updated mission
            robotpath = defaultdict(list)
            for r in robots:
                path_tuple = tuple(r.histo)
                robotpath[path_tuple].append(r.name)

            unique_paths_with_robots = [{'path': list(path), 'robots': robots} for path, robots in robotpath.items()]
            new_unique_paths = Class_and_functions.split_redundant_paths(unique_paths_with_robots, mission)
            # DEBUG CASE 3
            new_unique_paths = [
                {'path': ['site3', 'site5'], 'robots': ['robot0', 'robot4']},
                {'path': ['site7'], 'robots': ['robot5', 'robot6']},
                {'path': ['site2', 'site6'], 'robots': ['robot2', 'robot5', 'robot6']}
            ]

            for i, path_info in enumerate(new_unique_paths):
                print("rawpath {}: ".format(str(i)), path_info['path'], "robots: ", path_info['robots'])

            # sequential_links = Class_and_functions.find_sequential_links(new_unique_paths, mission)
            sequential_links = [(1, 2)] 

            for r in robots:
                robot_states[r.name] = {
                    "site": r.site,
                    "loc": r.loc,
                    "position": r.poi,
                    "conf": "waterconf" if r.medium ==-1 else "airconf",
                    "time": 792.16
                }

            active_robot_names = {r.name for r in robots}

            # Step 2: Remove robots not in that list
            for name in list(robot_states.keys()):
                if name not in active_robot_names:
                    del robot_states[name]

            print(robot_states)

            stn = Class_and_functions.build_STN(
                new_unique_paths,
                sequential_links,
            )

            # ✅ Then return or continue below normally (skip report reloading part)
            executed_paths = []
        else:
            # Extract relevant report fields
            best_alloc = report["best_alloc"]
            arms_paths = report["arms_paths"]
            path_viability = report.get("path_viability", {})
            fixed_paths = [str(pid) for pid in report.get("fixed_paths", [])]
            sequential_links_dict = report.get("sequential_links", {})
            sequential_links = [
                (pred, dep)
                for dep, preds in sequential_links_dict.items()
                for pred in preds
            ]

            
            valid_path_ids = {str(p["id"]) for p in arms_paths}

            # Determine executed paths (fixed + unaffected)
            unaffected_paths = [
                pid for pid, data in path_viability.items()
                if pid in valid_path_ids and data.get("status") == "unaffected" and data.get("valid")
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
                print(f"rawpath {p['id']}: {p['path']} robots: {p['robots']}")

            print("sequential_links: ", sequential_links, "\n")

            # Identify robots involved in executed (fixed + unaffected) paths
            executed_robots = set()
            for p in new_unique_paths:
                if p["id"] in executed_paths:
                    executed_robots.update(p["robots"])

            t_alloc_full_0 = time.perf_counter()

            # Phase 1: update robot_states using merged plan files for fixed paths
            for path_id in fixed_paths:
                plan_file_path = f"/tmp/plan_output/subproblems/PATH_{path_id}_mergedPLAN.txt"

                if not os.path.exists(plan_file_path):
                    print(f"⚠️ Skipping {path_id}: Plan file not found at {plan_file_path}")
                    continue

                print(f"🔍 Processing merged plan for path {path_id}: {plan_file_path}")

                # Update the robot_states dictionary in-place
                robot_states, _, _ = Class_and_functions.p_plan(plan_file_path, robot_states, mission)

                # for r in best_alloc[str(path_id)]:
                #     print(f"🆙 Robot {r} after plan parse: {robot_states[r]}")

            # Phase 2 (clean): Only update the robots in report["failed_robots"] using world_info
            with open("/tmp/world_info.json") as f:
                world_info = json.load(f)

            failed_robots = report.get("failed_robots", [])

            for r in failed_robots:
                matching_robot = next((rob for rob in world_info["robots"] if rob["name"] == r), None)
                if matching_robot:
                    print(f"🌍 Overriding state for failed robot {r} using world_info")
                    robot_states[r] = {
                        "site": matching_robot["site"],
                        "loc": matching_robot["loc"],
                        "position": matching_robot["poi"],
                        "conf": "waterconf" if matching_robot["medium"] == -1 else "airconf",
                        "time": 397.294
                    }


            # # Final print
            # print("\n📦 Final reconstructed robot states (merged plan + world fallback):")
            # for r, s in robot_states.items():
            #     print(f"{r}: pos={s['position']} | conf={s['conf']} | site={s['site']} | loc={s['loc']} | t={s['time']:.1f}")

            stn = Class_and_functions.build_STN(
                new_unique_paths,
                sequential_links,
                executed_paths=executed_paths
            )

    else:
        print("🆕 No validation report provided → running full mission planning")
        # We establish the best assignement for this mission with IROS2024 solution
        print("NB SITES: ",len(mission.sites[1::]),",NB ROBOTS: ", len(mission.robots),",NB OBJECTIVES: ",sum([len(s.poi) for s in mission.sites[1::]])-len(mission.sites[1::]), "MIN NB CLUSTER: ",min_nb_cluster)
        clusters, allocationscenario = Class_and_functions.getbestassignfrommission(mission=mission,minnbofcluster=min_nb_cluster)

        print("Clusters:")
        for c in clusters:
            print(c.name)
            for s in c.sites:
                print(s.name)
            print("\n")
            
        # Dictionary to store the paths and their corresponding robots
        robotpath = defaultdict(list)
        print("Robot assignment:")
        for r in mission.robots:
            print(r.name, "assigned to", r.histo)
            path_tuple = tuple(r.histo)  # Convert list to tuple to use as a dict key
            # Append the robot name to the path key in robotpath dictionary
            robotpath[path_tuple].append(r.name)

        # Now format the results into a list that includes each unique path with its robots
        unique_paths_with_robots = [{'path': list(path), 'robots': robots} for path, robots in robotpath.items()]

        # We split redundant paths into severals
        new_unique_paths = Class_and_functions.split_redundant_paths(unique_paths_with_robots, mission)

        print("\n")
        # print("Unique paths in plan:")
        for i, path_info in enumerate(new_unique_paths):
            print("rawpath {}: ".format(i),path_info['path'], "robots: ",path_info['robots'])

        t_alloc_full_0 = time.perf_counter()

        # Following the robots assignements in the unique paths we can find the sequential links between paths
        sequential_links = Class_and_functions.find_sequential_links(new_unique_paths, mission) 
        print("sequential_links: ", sequential_links, "\n")

        robot_states = {
            r.name: {"site": "base", "position": "cpbase", "conf": "groundconf", "loc": mission.sites[0].center, "time": 0}
            for r in mission.robots
        }  # Store last known state of each robot globally

        stn = Class_and_functions.build_STN(
            new_unique_paths,
            sequential_links,
        )
        # print(export_networkx_stn(stn))
        # Class_and_functions.draw_STN(stn)
        # Class_and_functions.extract_paths_with_dependencies(stn)


    
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
    

    # Class_and_functions.draw_STN(stn)

    t_STN_0 = time.perf_counter()
    

    while not all(stn.nodes[str(p)]["executed"] for p in stn.nodes if p not in ["Start", "End"]):  # Excluding "Start" and "End" nodes
        executable_paths = Class_and_functions.get_executable_paths(stn)
        # print(executable_paths)

        for path_idx in executable_paths:
            print("Solving path : ", path_idx)
            sync_created = False
            path_info = stn.nodes[str(path_idx)]
            pathplan = []
            path = path_info["sites"]
            robots = path_info["robots"]
            if "End" in stn.successors(str(path_idx)) and mission.sites[0].name not in path:
                path.append(mission.sites[0].name)

            # print(f"\n🚀 Executing path {path_idx}: {path} with robots {robots}")

            # Retrieve last known state for each robot
            robot_state = {robot: robot_states[robot] for robot in robots}
            previous_robotstate = {r: dict(robot_states[r]) for r in robots}

            # **Step 1: Handle Multi-Predecessor Synchronization (Dynamic "Goto" Paths)**
            predecessors = list(stn.predecessors(str(path_idx)))
            site_obj = next((s for s in mission.sites if s.name == path[0]), None)

            # === STEP 1: Synchronization (Multi-predecessor case) loop ===
            if len(predecessors) > 1 and not any(robot_state[robot]["site"] == path[0] for robot in robots):
                travel_time = {}
                finishing_times = {}
                arrival_times = {}

                for p in predecessors:
                    for robot in stn.nodes[str(p)]["robots"]:
                        if robot in robots:  # ✅ Only consider robots assigned to path_idx
                            # travel_time = Class_and_functions.distance(robot_states[robot]["loc"], site_obj.center) / 1.5
                            travel_time[robot] = Class_and_functions.distance(robot_states[robot]["loc"], site_obj.center) / 1.5
                            finishing_times[robot] = stn.nodes[str(p)]["latest_finish"]
                            arrival_times[robot] = stn.nodes[str(p)]["latest_finish"] + travel_time[robot]

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
                            for robot in stn.nodes[str(p)]["robots"]:
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
                    sync_robots = [robot for robot in stn.nodes[str(p)]["robots"] if robot in robots]  # Only assigned robots
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
                    stn.remove_edge(str(p), path_idx)  # Remove direct link

                    # Add sync transition: travel only
                    sync_duration = round(latest_finish - earliest_start, 2)
                    travel_only = round(earliest_start - finishing_times[sync_robots[0]], 2)  # Could average if you prefer

                    stn.add_edge(str(p), sync_path_id, min_time_gap=travel_only)
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
                prblm_path = Class_and_functions.GenerateAdaptedProblem(mission, problem_file, robot_state, path[i], goal_type)
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
                # print(robot_state, plan_end_time)
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

                stn.nodes[str(path_idx)]["sites"] = path1

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

                for succ in list(stn.successors(str(path_idx))):
                    edge_data = stn.get_edge_data(str(path_idx), succ)
                    stn.remove_edge(str(path_idx), succ)
                    stn.add_edge(sync_path_id, succ, **edge_data)

                stn.add_edge(str(path_idx), sync_path_id, min_time_gap=stn.nodes[str(path_idx)]["execution_time"])

                # print(f"📎 Splitting after execution: Path {path_idx} had {plan_lines} actions")
                # print(f"    🔹 {path1} (ID: {path_idx})")
                # print(f"    🔹 {path2} (ID: {sync_path_id})")

                # ✅ Rewind robot state based on path1 execution
                # print("🔁 Recomputing robot state for split path (only path1 executed)")

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
                stn.nodes[str(path_idx)]["executed"] = True
    
    # print(export_networkx_stn(stn))
    # Class_and_functions.draw_STN(stn, impact_colors={"0":"lightcoral", "sync_0_a":"coral", "3":"coral"})
    # Class_and_functions.draw_STN(stn)
    t_STN_1 = time.perf_counter()
    print("STN creation and PDDL solving time:", t_STN_1-t_STN_0)
    
    Class_and_functions.extract_paths_with_dependencies(stn)
    t_alloc_full_1 = time.perf_counter()
    print("full time pre-alloc:",t_alloc_full_1-t_alloc_full_0)
if __name__ == '__main__':
    main()
