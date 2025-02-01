# import os
# # stud = ["brute_force_vs_sites", "clusters_limits", "sites_vs_clusters", "trajectories_problem"]
# stud = ["clusters_limits"]
# Technique= ["Cdiv"] #Cdiv 
# for study in stud:
#     for batch in range(0,20):
#         for num in range(0,28):   
#             for tech in Technique:
#                 os.system("mkdir -p /home/virgile/PHD/ProblemOutput/"+ study +"/batch{0}".format(batch) + "/problem{0}".format(num) + tech)
#                 # os.system("mkdir -p /home/virgile/PHD/Ros2PLAN_ws/src/TMMS/"+ study +"/batch{0}".format(batch))


import re
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

import re
from typing import List, Tuple
import numpy as np
from dataclasses import dataclass, field


@dataclass
class Robot():
    """ dataclass containing information about a robot """
    name:str = ""
    """ Name of the robot, e.g: "robot0" """
    loc:tuple[float,float,float] = (0,0,0)
    """ Coordinates of the robot in 3D """
    medium:int = 0
    """ Current medium of the robot """
    histo:list = field(default_factory=list)
    """ Memory of all the sites that the robot visited """
    currentpathcost:float = 0
    """ Current cost from previous path """
    poi:str = "cpbase"
    """ Current Position in point of interest """
    site:str = "base"
    """ Current Position in Site """
    energy:float = 10000
    """ Current energy of the robot """
    
    # @property 
    # def x(self): 
    #     """ test """
    #     return self.Loc[0]

@dataclass
class Site():
    """ A class containing information for a Site """
    poi:list 
    """ List of the Point of Interest contained in the site """
    robots:list = field(default_factory=list)
    """ robots in the site """
    name:str = "base"
    """ Name of the site, e.g: "site0" """
    center:tuple[float,float,float] = (0,0,0)
    """ Coordinates of the center of the site """
    size:float = 50
    """ Size of side of the square site """

@dataclass 
class Cluster_class():
    """ A class containing information for a Cluster of sites """
    robots:list = field(default_factory=list)
    """ robots in the site """
    sites:list = field(default_factory=list)
    """ The list of sites class """
    name:str = "cluster1"
    """ Name of the site, e.g: "site0" """
    center:tuple[float,float,float] = (0,0,0)
    """ Coordinates of the center of the site """
    site_assign_costs:dict[int, float] = field(default_factory=dict)
    """ Dictionary mapping number of robots to cost """
    bestscenario:dict[int, tuple] = field(default_factory=dict)
    """ The dict of robot attribution in best case for this cluster depending on number of robots"""

    @property
    def centroid_of_sites_2d(self):
        points=[]
        for s in self.sites:
            points.append(s.center)
        x_center = sum(point[0] for point in points) / len(points)
        y_center = sum(point[1] for point in points) / len(points)
        self.center = (x_center, y_center)

@dataclass
class Poi():
    """ A class containing information for a point of interest """
    mediums:list 
    """ List of mediums that are required for accessing this poi """
    name:str = "cpbase"
    """ Name of the point of interest, e.g: "sp1", "pp1","bp" """
    loc:tuple[float,float,float] = (0,0,0)
    """ Coordinates of the Poi in 3D  """
    typepoi:str = "transition"
    """ Type of poi, e.g: "transition", "sample", "survey" """

@dataclass
class Mission():
    """ A class containing all the informations for a mission """
    sites:list = field(default_factory=list)
    """ The list of sites class """
    robots:list = field(default_factory=list)
    """ The list of robot class """
    objective:str = "explore"
    """ The objective for the mission, e.g: "assess", "sample", "explore" """
    arenasize:int = 200
    """ Size of side of the square arena """
    sitesize:list = field(default_factory=list)
    """ Size of side of the square site """


@dataclass
class ScenarioAttribution():
    """ A class containing all the informations for a scenario attribution  """
    clusterinscenario:list
    """ the list of cluster or sites in the scenario """
    scenario:tuple 
    """ The tuple of robot attribution in scenario """
    numrobots:int = 2
    """ the number of robot in scenario """
    totalcost:float = 0.0
    """ The total cost of the scenario """
    costmatrix:np.ndarray = np.array([])
    """ the cost matrix linked to the scenario clusters """



def parse_pddl_to_mission(filepath: str) -> Mission:
    """
    Parse a PDDL problem file to generate a Mission dataclass.
    """
    with open(filepath, "r") as file:
        lines = file.readlines()

    mission = Mission()
    current_section = None

    # Temporary storage for objects and states
    robots = {}
    sites = {}
    pois = {}

    for line in lines:
        line = line.strip()

        if line.startswith(";"):
            continue  # Skip comments

        # Detect sections
        if line.startswith("(:objects"):
            current_section = "objects"
            continue
        elif line.startswith("(:init"):
            current_section = "init"
            continue
        elif line.startswith("(:goal"):
            current_section = "goal"
            continue
        elif line == ")":
            current_section = None
            continue

        if current_section == "objects":
            # Parse robots, sites, and POIs
            if "- robot" in line:
                robot_names = line.replace("- robot", "").strip().split()
                for name in robot_names:
                    robots[name] = Robot(name=name)
            elif "- site" in line:
                site_names = line.replace("- site", "").strip().split()
                for name in site_names:
                    sites[name] = Site(name=name, poi=[])
            elif "- pointofinterest" in line:
                poi_names = line.replace("- pointofinterest", "").strip().split()
                for name in poi_names:
                    pois[name] = Poi(name=name, mediums=[])

        elif current_section == "init":
            # Parse initial states
            if line.startswith("(at "):
                match = re.match(r"\(at (\w+) (\w+)\)", line)
                if match:
                    robot_name, poi_name = match.groups()
                    robots[robot_name].poi = poi_name
            elif line.startswith("(at_site "):
                match = re.match(r"\(at_site (\w+) (\w+)\)", line)
                if match:
                    robot_name, site_name = match.groups()
                    robots[robot_name].site = site_name
            elif line.startswith("(groundconf "):
                match = re.match(r"\(groundconf (\w+)\)", line)
                if match:
                    robots[match.group(1)].medium = 0
            elif line.startswith("(airconf "):
                match = re.match(r"\(airconf (\w+)\)", line)
                if match:
                    robots[match.group(1)].medium = 1
            elif line.startswith("(waterconf "):
                match = re.match(r"\(waterconf (\w+)\)", line)
                if match:
                    robots[match.group(1)].medium = -1
            elif line.startswith("(partofsite "):
                match = re.match(r"\(partofsite (\w+) (\w+)\)", line)
                if match:
                    poi_name, site_name = match.groups()
                    pois[poi_name].typepoi = "unknown"
                    sites[site_name].poi.append(pois[poi_name])
            elif line.startswith("(transition_poi "):
                match = re.match(r"\(transition_poi (\w+)\)", line)
                if match:
                    pois[match.group(1)].typepoi = "transition"
            elif line.startswith("(survey_poi "):
                match = re.match(r"\(survey_poi (\w+)\)", line)
                if match:
                    pois[match.group(1)].typepoi = "survey"
            elif line.startswith("(sample_poi "):
                match = re.match(r"\(sample_poi (\w+)\)", line)
                if match:
                    pois[match.group(1)].typepoi = "sample"

        elif current_section == "goal":
            # Parse goals
            if "(assessed " in line:
                match = re.match(r"\(assessed (\w+)\)", line)
                if match:
                    poi_name = match.group(1)
                    # Add a goal or set objective to "assess"
                    mission.objective = "assess"

    # Build Mission from parsed data
    mission.sites = list(sites.values())
    mission.robots = list(robots.values())

    return mission


# Example Usage
mission = parse_pddl_to_mission("MMtest.pddl")
print(mission)

