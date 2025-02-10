import sys
import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import networkx as nx
import numpy as np
import subprocess
from collections import defaultdict
from pathlib import Path
import time
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

# Import your custom module
import Class_and_functions

# Get the path for the problem, domain and parameters
nb_cluster = int(sys.argv[3])
domain_file = sys.argv[1]

# (:requirements :strips :typing :fluents :negative-preconditions :timed-initial-literals :disjunctive-preconditions :durative-actions :universal-preconditions )

def check_domain_arms(domain_file):
    # Function checking for the timed-initial-literals requirement that is not yet added to plansys2 pddl parser
    with open(domain_file, 'r') as file:
        lines = file.readlines()

    # Check if ":timed-initial-literals" is in the second line
    if ":timed-initial-literals" not in lines[1]:
        print("Adding missing requirement ':timed-initial-literals' to domain file.")

        # Insert ":timed-initial-literals" into the requirements section
        if ":requirements" in lines[1]:
            lines[1] = lines[1].strip()[:-1] + ":timed-initial-literals)\n"  # Append the requirement properly
        else:
            lines.insert(1, "(:requirements :timed-initial-literals)\n")  # Create a new requirements line

        # Write the new domain file to /tmp/domain-optic.pddl
        new_domain_file = "/tmp/domain-optic.pddl"
        with open(new_domain_file, 'w') as file:
            file.writelines(lines)

        return new_domain_file  # Return the new domain file path

    print("Domain file already contains ':timed-initial-literals'.")
    return domain_file  # Use the original file if no modification was needed

domain_file = check_domain_arms(domain_file)
problem_file = sys.argv[2]
output_file = base_output_dir / "arms_output.txt"

# For subproblems:
def main():

    # We get the mission data class from the problem file
    mission = Class_and_functions.recover_mission_from_json("/tmp/world_info.json")

    # # We establish the best assignement for this mission with IROS2024 solution
    # print("NB SITES: ",len(mission.sites[1::]),",NB ROBOTS: ", len(mission.robots),",NB OBJECTIVES: ",sum([len(s.poi) for s in mission.sites[1::]])-len(mission.sites[1::]), "MIN NB CLUSTER: ",nb_cluster)
    # clusters, allocationscenario = Class_and_functions.getbestassignfrommission(mission=mission,minnbofcluster=nb_cluster)

    # print("Clusters:")
    # for c in clusters:
    #     print(c.name)
    #     for s in c.sites:
    #         print(s.name)
    #     print("\n")

    # totaldistancetime=[]

    # # Dictionary to store the paths and their corresponding robots
    # robotpath = defaultdict(list)

    # print("Robot assignement:")
    # for r in mission.robots:
    #     print(r.name, "assigned to", r.histo)

    #     # Calculate distances and prepare the histo with 'base' at start and end
    #     totaldistancetime.append(Class_and_functions.gettotaldistances(r.histo, mission.sites))
    #     path_with_base = ['base'] + r.histo + ['base']
    #     path_tuple = tuple(path_with_base)  # Convert list to tuple to use as a dict key

    #     # Append the robot name to the path key in robotpath dictionary
    #     robotpath[path_tuple].append(r.name)

    # print("\n")
    # print("total max time of agents movement", max(totaldistancetime))

    # # Now format the results into a list that includes each unique path with its robots
    # unique_paths_with_robots = [{'path': list(path), 'robots': robots} for path, robots in robotpath.items()]
    unique_paths_with_robots =    [{'path': ['base', 'site7', 'site3', 'base'], 'robots': ['robot0', 'robot1']}, {'path': ['base', 'site8', 'site4', 'base'], 'robots': ['robot2', 'robot3']}, {'path': ['base', 'site2', 'site6', 'base'], 'robots': ['robot4', 'robot5']}, {'path': ['base', 'site9', 'site5', 'base'], 'robots': ['robot6', 'robot7']}, {'path': ['base', 'site1', 'base'], 'robots': ['robot8', 'robot9']}] 

    mergedplans = []
    for i,path_info in enumerate(unique_paths_with_robots):
        print("path {}: ".format(i),path_info['path'], "robots: ",path_info['robots'])
        pathplan = []
        path = path_info['path']
        robot_state = {i: {} for i in path_info['robots']}
        robot_names = "".join(robot_state.keys())
        

        for i in range(len(path)-1):
            
            # We generate the problem for the current goal in the path
            next_site = next((site for site in mission.sites if site.name == path[i+1]), None)
            current_site = next((site for site in mission.sites if site.name == path[i]), None)
            prblm_path= Class_and_functions.GenerateAdaptedProblem(problem_file, robot_state, current_site,next_site)
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
            # Get the current state from the plan
            robot_state = Class_and_functions.p_plan("{0}_PLAN.txt".format(prblm_path),robot_state)

        Class_and_functions.merge_for_single_path(pathplan, "{0}_mergedPLAN.txt".format(str(subproblems_dir) + "/" +robot_names), domain_file, Path(problem_file))
        mergedplans.append(Path("{0}_mergedPLAN.txt".format(str(subproblems_dir) + "/" + robot_names)))

    ## Merge all plans
    Class_and_functions.merge_for_multi_path(mergedplans, "{0}/mergedplan.txt".format(base_output_dir_str), domain_file, Path(problem_file))
    

if __name__ == '__main__':
    main()
