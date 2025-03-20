import sys
from itertools import combinations
import os
import subprocess
import time
import scipy.stats as scistat
from collections import defaultdict
from pathlib import Path
from ament_index_python.packages import get_package_prefix

# Dynamically add the directory to the Python path
# Define the output paths dynamically
base_output_dir = Path("/tmp/plan_output/")
base_output_dir_str ="/tmp/plan_output/"
subproblems_dir = base_output_dir / "subproblems"

# Ensure the directories exist
subproblems_dir.mkdir(parents=True, exist_ok=True)

script_dir = os.path.dirname(__file__)  # Get the directory where the script is located
parent_dir = os.path.dirname(script_dir)  # Navigate to the parent directory if needed
external_solver_dir = os.path.join(get_package_prefix("arms_plan_solver"), "lib", "arms_plan_solver", "external_solver")
optic_cplex_path = os.path.join(external_solver_dir, "optic-cplex")
sys.path.append(parent_dir)  # Add the parent directory to sys.path
sys.path.append(script_dir + "/functions")

import Class_and_functions  # Import ARMS Toolbox functions

#  Parameters for planning
min_nb_cluster = int(sys.argv[3])
# mode = sys.argv[4]
# paths = sys.argv[5:]
domain_file = sys.argv[1]

# (:requirements :strips :typing :fluents :negative-preconditions :timed-initial-literals :disjunctive-preconditions :durative-actions :universal-preconditions )



domain_file = Class_and_functions.check_domain_arms(domain_file)
problem_file = sys.argv[2]
output_file = base_output_dir / "arms_output.txt"

# For subproblems:
def main():
    """ 
    - Assigns robots to paths based on site clusters
    - Ensures robots in the same locations are properly grouped
    - Generates and solves planning problems
    - Merges plans correctly 
    """

    # We get the mission data class from the problem file
    mission = Class_and_functions.recover_mission_from_json("/tmp/world_info.json")

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

    print("Unique paths in plan:")
    for i, path_info in enumerate(new_unique_paths):
        print("path {}: ".format(i),path_info['path'], "robots: ",path_info['robots'])

    t_alloc_full_0 = time.perf_counter()
    # Following the robots assignements in the unique paths we can find the sequential links between paths
    sequential_links = Class_and_functions.find_sequential_links(new_unique_paths, mission) 

    # With the sequential links we can initialize a STN of paths
    stn = Class_and_functions.build_STN(new_unique_paths, sequential_links)
    
    # mergedplans = []
    robot_states = {
        r.name: {"site": "base", "position": "cpbase", "conf": "groundconf", "loc": mission.sites[0].center, "time": 0}
        for r in mission.robots
    }  # Store last known state of each robot globally
    # print("THE ROBOT STATES", robot_states, "\n")

    while not all(stn.nodes[p]["executed"] for p in stn.nodes if p not in ["Start", "End"]):  # Excluding "Start" and "End" nodes
        executable_paths = Class_and_functions.get_executable_paths(stn)
        # print("🔹 Executable paths:", executable_paths)
        # print([n for n in stn.nodes()])
        # print([list(stn.predecessors(n)) for n in stn.nodes()])
        # print([list(stn.successors(n)) for n in stn.nodes()])

        for path_idx in executable_paths:
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

            # **Step 1: Handle Multi-Predecessor Synchronization (Dynamic "Goto" Paths)**
            predecessors = list(stn.predecessors(path_idx))

            site_obj = next((s for s in mission.sites if s.name == path[0]), None)
            # ✅ **New Condition: Check if Any Robot is Already at path[0]**
            if len(predecessors) > 1 and not any(robot_state[robot]["site"] == path[0] for robot in robots):
                # print(f"⚠️ Path {path_idx} has multiple predecessors: {predecessors}")
                # Get the **latest** finishing time among all predecessors
                # print("look at predecessors:", [stn.nodes[p]["latest_finish"] for p in predecessors])

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

                    # Add Sync Node to STN (one per predecessor path)
                    stn.add_node(sync_path_id, sites=[path[0]], robots=sync_robots, earliest_start=0, latest_finish=float('inf'), execution_time=0, executed=False)

                    # Adjust STN edges
                    stn.remove_edge(p, path_idx)  # Remove direct link
                    stn.add_edge(p, sync_path_id, min_time_gap=finishing_times[sync_robots[0]])  # Add sync transition
                    stn.add_edge(sync_path_id, path_idx, min_time_gap=arrival_times[sync_robots[0]])  # Continue to main path


                sync_created = True  # Mark that we modified the STN
                
            # A sync path has been created so we need to execute it first
            if sync_created:
                break

            # **Step 2: Solve Each Step in the Path**
            for i in range(len(path)):
                # print(f"🔹 Solving: {[r for r in robot_state.keys()]} -> {path[i]}")

                # Define goal type
                goal_type = "goto" if path[i] == "base" or "sync" in str(path_idx) else "solving"

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

            for robot in robots:
                robot_states[robot] = robot_state[robot]  # Save robot's latest known state

            # print("THE ROBOT STATES", robot_states, "\n")

            merged_path_file = f"{subproblems_dir}/PATH_{path_idx}_mergedPLAN.txt"
            Class_and_functions.merge_for_single_path(pathplan, merged_path_file, domain_file, Path(problem_file))
            # mergedplans.append(Path(merged_path_file))

            # Update STN with execution times
            Class_and_functions.update_STN_temporal_links(stn, path_idx, max_execution_time)
            # **Mark the Node as Executed Instead of Removing It**
            stn.nodes[path_idx]["executed"] = True

    # Class_and_functions.draw_STN(stn)
    t_alloc_full_1 = time.perf_counter()
    Class_and_functions.extract_paths_with_dependencies(stn)
    print(t_alloc_full_1-t_alloc_full_0)
if __name__ == '__main__':
    main()
