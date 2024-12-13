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

# Dynamically add the directory to the Python path
script_dir = os.path.dirname(__file__)  # Get the directory where the script is located
parent_dir = os.path.dirname(script_dir)  # Navigate to the parent directory if needed
sys.path.append(parent_dir)  # Add the parent directory to sys.path
sys.path.append(script_dir + "/functions")

# Import your custom module
import Class_and_functions

# Get the path for the problem, domain and parameters
nb_cluster = int(sys.argv[3])
domain_file = sys.argv[1]
problem_file = sys.argv[2]
output_file = script_dir + "output/output.txt"

def main():

    # We get the mission data class from the problem file
    mission = Class_and_functions.getdataclassfrompddl(problem_file)

    # We establish the best assignement for this mission with IROS2024 solution
    print("NB SITES: ",len(mission.sites[1::]),",NB ROBOTS: ", len(mission.robots),",NB OBJECTIVES: ",sum([len(s.poi) for s in mission.sites[1::]])-len(mission.sites[1::]), "MIN NB CLUSTER: ",nb_cluster)
    clusters, allocationscenario = Class_and_functions.getbestassignfrommission(mission=mission,minnbofcluster=nb_cluster)

    print("Clusters:")
    for c in clusters:
        print(c.name)
        for s in c.sites:
            Class_and_functions.generateNewMissionRobotProblems(s, "output/subproblems/")
            print(s.name)
        print("\n")

    
    totaldistancetime=[]

    # Dictionary to store the paths and their corresponding robots
    robotpath = defaultdict(list)

    print("Robot assignement:")
    for r in mission.robots:
        print(r.name, "assigned to", r.histo)

        # Calculate distances and prepare the histo with 'base' at start and end
        totaldistancetime.append(Class_and_functions.gettotaldistances(r.histo, mission.sites))
        path_with_base = ['base'] + r.histo + ['base']
        path_tuple = tuple(path_with_base)  # Convert list to tuple to use as a dict key

        # Append the robot name to the path key in robotpath dictionary
        robotpath[path_tuple].append(r.name)

    print("\n")
    print("total max time of path", max(totaldistancetime))

    # Now format the results into a list that includes each unique path with its robots
    unique_paths_with_robots = [{'path': list(path), 'robots': robots} for path, robots in robotpath.items()]
    
    ## Solve the sites
    for c in clusters:
        for s in c.sites:
            print(s.name)
            # multithreading process Queue 
            try:
                OPTIC_command = "timeout -v 1m ./optic-cplex -N {0} output/subproblems/{1}.pddl".format(domain_file, s.name)
                time_output_file = "output/subproblems/{0}time_output.txt".format(s.name)
                PDDL_outputfile = "output/subproblems/{0}problemPLAN.txt".format(s.name)

                # Combine the python command with /usr/bin/time, redirecting /usr/bin/time output to its file
                command = f"/usr/bin/time -v {OPTIC_command} > {PDDL_outputfile} 2> {time_output_file}"

                # Execute the combined command
                subprocess.run(command, shell=True)
                Class_and_functions.trim_plan_file("output/subproblems/{0}problemPLAN.txt".format(s.name))
            except IndexError:
                print("INDEX ERROR")


        
            # Get information of current state of robot and create the return problem for the current site to go back to surface 
            robot_state = Class_and_functions.p_plan("output/subproblems/{0}problemPLAN.txt".format(s.name))
            Class_and_functions.generateReturnProblem("output/subproblems/{0}.pddl".format(s.name), "output/subproblems/{0}return.pddl".format(s.name), robot_state)
            
            # Solve the return problem
            try:
                OPTIC_command = "timeout -v 1m ./optic-cplex -N {0} output/subproblems/{1}return.pddl".format(domain_file, s.name)
                time_output_file = "output/subproblems/{0}returntime_output.txt".format(s.name)
                PDDL_outputfile = "output/subproblems/{0}returnproblemPLAN.txt".format(s.name)

                # Combine the python command with /usr/bin/time, redirecting /usr/bin/time output to its file
                command = f"/usr/bin/time -v {OPTIC_command} > {PDDL_outputfile} 2> {time_output_file}"

                # Execute the combined command
                subprocess.run(command, shell=True)
                Class_and_functions.trim_plan_file("output/subproblems/{0}returnproblemPLAN.txt".format(s.name))
            except IndexError:
                print("INDEX ERROR")
            ## Merge the sites and return plan
            # tx = time.perf_counter() 
            Class_and_functions.merge_for_single_path([Path("output/subproblems/{0}problemPLAN.txt".format(s.name)),Path("output/subproblems/{0}returnproblemPLAN.txt".format(s.name))], "output/subproblems/{0}mergedPLAN.txt".format(s.name), domain_file, Path("output/subproblems/{0}return.pddl".format(s.name)))
            # print(time.perf_counter() - tx)


    
    # Iterate through each path to create navigation problems
    t1 = time.perf_counter() 
    
    mergedplans=[]
    for path_info in unique_paths_with_robots:
        pathplan = []
        path = path_info['path']
        robots = path_info['robots']
        robot_names = "".join(robots)
        # Iterate through each site in the path to create navigation problems between consecutive sites
        for i in range(len(path) - 1):
            current_site = path[i]
            next_site = path[i + 1]
            ## Call the function to generate the navigation problem
            
            if current_site == "base":
                Class_and_functions.generateNavProblem(problem_file, "output/subproblems/navigation/nav{0}{1}.pddl".format(robot_names, current_site+next_site), robots, current_site, next_site)
            else:
                robot_state = Class_and_functions.p_plan("output/subproblems/{0}returnproblemPLAN.txt".format(current_site))
                Class_and_functions.generateNavProblemSites(problem_file, "output/subproblems/navigation/nav{0}{1}.pddl".format(robot_names, current_site+next_site), robot_state, current_site, next_site)
            

            ## Solve the Navigation problem
            try:
                OPTIC_command = "timeout -v 1m ./optic-cplex -N {0} output/subproblems/navigation/nav{1}{2}.pddl".format(domain_file, robot_names, current_site+next_site)
                time_output_file = "output/subproblems/navigation/nav{0}{1}time_output.txt".format(robot_names, current_site+next_site)
                PDDL_outputfile = "output/subproblems/navigation/nav{0}{1}PLAN.txt".format(robot_names, current_site+next_site)

                # Combine the python command with /usr/bin/time, redirecting /usr/bin/time output to its file
                command = f"/usr/bin/time -v {OPTIC_command} > {PDDL_outputfile} 2> {time_output_file}"

                # Execute the combined command
                subprocess.run(command, shell=True)
                Class_and_functions.trim_plan_file("output/subproblems/navigation/nav{0}{1}PLAN.txt".format(robot_names, current_site+next_site))
            except IndexError:
                print("INDEX ERROR")
                
            ## Merge the resulting solving plan and the navigation plan in order
            # print("Merging robotpath", robot_names, current_site, next_site)
            
            if next_site == "base":
                pathplan.append(Path("output/subproblems/navigation/nav{0}{1}PLAN.txt".format(robot_names, current_site+next_site)))
                mergedplans.append(Path("output/subproblems/{0}mergedPLAN.txt".format(robot_names)))
                print(current_site, next_site, "output/subproblems/navigation/nav{0}{1}PLAN.txt".format(robot_names, current_site+next_site))

            else:
                pathplan.append(Path("output/subproblems/navigation/nav{0}{1}PLAN.txt".format(robot_names, current_site+next_site)))
                pathplan.append(Path("output/subproblems/{0}mergedPLAN.txt".format(next_site)))
                print(current_site, next_site, "output/subproblems/navigation/nav{0}{1}PLAN.txt".format(robot_names, current_site+next_site), "output/subproblems/{0}mergedPLAN.txt".format(next_site))
        Class_and_functions.merge_for_single_path(pathplan, "output/subproblems/{0}mergedPLAN.txt".format(robot_names), domain_file, Path(problem_file))
    
        
    
    ## Merge all plans
    Class_and_functions.merge_for_multi_path(mergedplans, "output/mergedplan.txt", domain_file, Path(problem_file))
    

if __name__ == '__main__':
    main()
