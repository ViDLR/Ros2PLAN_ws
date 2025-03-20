
import os
import time
from dataclasses import dataclass, field
from math import sqrt
from random import randint, randrange, uniform


import json
from scipy.spatial.distance import cdist
from itertools import combinations_with_replacement, product
from collections import defaultdict

import matplotlib.pyplot as plt
import graphviz
import pandas as pd
import networkx as nx
import numpy as np
import PySimpleGUI as sg
from python_tsp.exact import solve_tsp_dynamic_programming
from scipy.optimize import linear_sum_assignment as hungarian
from matplotlib.lines import Line2D
from scipy.optimize import minimize
from sklearn.cluster import KMeans
from fractions import Fraction
from pathlib import Path
from typing import Any, List, Tuple
from unified_planning.plans import PlanKind
from unified_planning.plans import TimeTriggeredPlan
from unified_planning.io.pddl_reader import PDDLReader
from unified_planning.io.pddl_writer import PDDLWriter
from unified_planning.plot import plot_time_triggered_plan, plot_causal_graph, plot_stn_plan, plot_partial_order_plan


######################### CLASSES ########################

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
    total_completion_time:float = 0.0
    """ time to complete the site in assignement """

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
    total_completion_time:float = 0.0
    """ time to complete the cluster in assignement """

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


######################### COMMON FUNCTIONS ########################

#Plot a cube of water to represent the pool
def plot_site_pool(o = (0,0,0), size=(1,1,1), ax=None,**kwargs):
    """ Show a pool at a specific location in a 3D pyplot window """

    # Plotting a cube element at position pos
    l, w, h = size
    
    x = [[o[0] - 0.5*l, o[0] + 0.5*l, o[0] + 0.5*l, o[0] - 0.5*l, o[0]- 0.5*l],  
         [o[0] - 0.5*l, o[0] + 0.5*l, o[0] + 0.5*l, o[0] - 0.5*l, o[0]- 0.5*l],  
         [o[0] - 0.5*l, o[0] + 0.5*l, o[0] + 0.5*l, o[0] - 0.5*l, o[0]- 0.5*l],
         [o[0] - 0.5*l, o[0] + 0.5*l, o[0] + 0.5*l, o[0] - 0.5*l, o[0]- 0.5*l],]  
    y = [[o[1]-0.5*w, o[1]-0.5*w, o[1] + 0.5*w, o[1] + 0.5*w, o[1]-0.5*w],  
         [o[1]-0.5*w, o[1]-0.5*w, o[1] + 0.5*w, o[1] + 0.5*w, o[1]-0.5*w],  
         [o[1]-0.5*w, o[1]-0.5*w, o[1]-0.5*w, o[1]-0.5*w, o[1]-0.5*w],          
         [o[1] + 0.5*w, o[1] + 0.5*w, o[1] + 0.5*w, o[1] + 0.5*w, o[1] + 0.5*w]]   
    z = [[o[2]-0.5, o[2]-0.5, o[2]-0.5, o[2]-0.5, o[2]-0.5],
         [o[2] + h, o[2] + h, o[2] + h, o[2] + h, o[2] + h],   
         [o[2]-0.5, o[2]-0.5, o[2] + h, o[2] + h, o[2]-0.5],               
         [o[2]-0.5, o[2]-0.5, o[2] + h, o[2] + h, o[2]-0.5]] 
    if ax !=None:
        ax.plot_transition(np.array(x), np.array(y), np.array(z), rstride=1, cstride=1, **kwargs)

def distance(x = (0,0,0),y = (0,0,0)):
    """ Return the distance between two points """

    d = 0
    for i in range(len(x)):
        d += (y[i]-x[i])**2
    return round(sqrt(d),2)

def closeness(v1 = (0,0,0),v2 = (0,0,0),thresh=0.1):
    """ Return a boolean True if two points are closer than a given threshold """

    return distance(v1,v2) < thresh

def getlistusedRgroups(robots=[]):
    """ Return a list of possible teams for allocation given a number of robots """

    listindexR = [r for r in range(2,len(robots)+1)]
    for r in listindexR:
        if(len(robots)//r)>1 or (len(robots)%r >1) or (len(robots)%r ==0):
            continue
        else:
            listindexR.remove(r)
    # print(listindexR)
    return listindexR

def getcenterpoiofsites(sites=[]):
    """ Return a list all center points given a list of clusters (sites) """

    list_of_sites_center = []
    for s in sites:
        list_of_sites_center.append(s.poi[0].loc)
    return list_of_sites_center


def gettotaldistances(histo, sites):
    """ Return the maximum tour distance for a specific scenario """

    totalduration = 0
    sitesvisited = [sites[0]]
    
    for name in histo:
        sitesvisited.append(sites[int(name.partition("site")[2])])
        
    for s in range(len(sitesvisited)):
        if s!=0:
            totalduration += distance(sitesvisited[s].center, sitesvisited[s-1].center)

    totalduration = totalduration/1.5
    return totalduration

######################### UI AND SHOW FUNCTIONS ########################

def createwindow():
    """ Create a dynamic window for UI purposes"""

    layout = [[sg.Text('Define parameters for the simulation', size=(30, 1), font=("Helvetica", 25), text_color='black')],   
        [sg.Text('Choose the scenario', size=(35, 1))],     
        [sg.Radio('Scenario 1', "SC", default=True), sg.Radio('Scenario 2', "SC"), sg.Radio('Scenario 3', "SC")], 
        [sg.Text('Choose sliders: Number of robots, Number of sites, Number of beacon robot', size=(80, 1))],
        [sg.Text('                       Distance max de communication', size=(40, 1))],       
        # sg.Listbox(values=['Number of robots', 'Number of sites', 'Number of beacon robot'], size=(30, 6)),      
        [sg.Slider(range=(1, 10), orientation='v', size=(10, 20), default_value=3),      
        sg.Slider(range=(1, 10), orientation='v', size=(10, 20), default_value=3),      
        sg.Slider(range=(0, 2), orientation='v', size=(10, 20), default_value=1),
        sg.Slider(range=(1, 10), orientation='v', size=(10, 20), default_value=5)],   
        [sg.Submit(), sg.Cancel(), sg.Button('Random', button_color=('white', 'green'))]]     

    window = sg.Window('Simulation GUI', layout, auto_size_text=True, default_element_size=(40, 1))

    return window


def showmissionsites(mission=Mission(sites=[],robots=[])):
    """ Show the mission sites in a plot window """

    color = iter(plt.cm.brg(np.linspace(0, 1, len(mission.sites))))
    for s in range(1,len(mission.sites)):
        c = next(color)
        for pt in mission.sites[s].poi:
            if pt.name.startswith("cp"):
                plt.scatter(pt.loc[0], pt.loc[1],marker = "o", color=c, label="{0}".format(mission.sites[s].name))
                plt.text(pt.loc[0], pt.loc[1], "{}".format(mission.sites[s].name), size=15, zorder=100,color="k")
            elif pt.name.startswith("pp"):
                plt.scatter(pt.loc[0], pt.loc[1],marker = "x", color=c)

            
            elif pt.name.startswith("sp"):
                plt.scatter(pt.loc[0], pt.loc[1],marker = "o", color=c)

    plt.scatter(mission.sites[0].poi[0].loc[0], mission.sites[0].poi[0].loc[1],marker = "o", color="k", label="base")
    plt.text(mission.sites[0].poi[0].loc[0], mission.sites[0].poi[0].loc[1], "base", size=15, zorder=100,color="k")
    plt.xlim([-mission.arenasize, mission.arenasize])
    plt.ylim([-mission.arenasize, mission.arenasize])
    plt.legend()
    plt.show() 

def showclusters(mission, clusterslist, base=Site(poi=[], robots=[])):
    """ Show the mission cluster in a plot window """

    color = iter(plt.cm.brg(np.linspace(0, 1, len(clusterslist))))
    idx=0
    for cluster in clusterslist:
        c = next(color)
        Xr = np.array([base.poi[0].loc[0]])
        Yr = np.array([base.poi[0].loc[1]])
        
        for s in cluster.sites:
            for pt in s.poi:

                if pt.name.startswith("cp"):
                    Xr = np.append(Xr, pt.loc[0])
                    Yr = np.append(Yr, pt.loc[1])
                    plt.scatter(pt.loc[0], pt.loc[1],marker = "o", color=c, label="{0}".format(s.name))
                    plt.text(pt.loc[0], pt.loc[1], "{}".format(s.name), size=15, zorder=100,color="k")
                    # plt.scatter(pt.loc[0], pt.loc[1],marker = "v", color=c, label=" cluster {0} {1}".format(cluster, pt.name[2::]))
                elif pt.name.startswith("pp"):
                    plt.scatter(pt.loc[0], pt.loc[1],marker = "x", color=c)
                elif pt.name.startswith("sp"):
                    plt.scatter(pt.loc[0], pt.loc[1],marker = "o", color=c)
        
        # Xr = np.append(Xr, base.poi[0].loc[0])
        # Yr = np.append(Yr, base.poi[0].loc[1])
        # plt.plot(Xr,Yr,marker = '.', color=c, label='cluster {0} order'.format(idx))
        idx+=1
    
    plt.scatter(base.poi[0].loc[0], base.poi[0].loc[1],marker = "o", color="k", label="base")
    plt.text(base.poi[0].loc[0], base.poi[0].loc[1], "base", size=15, zorder=100,color="k")
    # plt.xlim([-mission.arenasize, mission.arenasize])
    # plt.ylim([-mission.arenasize, mission.arenasize])
    plt.xlabel("Mission axis x arena size in m", fontsize="15")
    plt.ylabel("Mission axis y arena size in m", fontsize="15")
    # plt.legend()
    plt.show()
    
######################### INITIALIZING FUNCTIONS ########################

def Random_Sites(mission=Mission(sites=[],robots=[]),nbs = int(), sitessize= int()):
    """Create a number of sites in a mission scenario"""

    # Initialize first site as base
    base = Site(name="base",center=(round(uniform(-mission.arenasize //2,mission.arenasize //2),1), round(uniform(-mission.arenasize //2,mission.arenasize //2),1), 0), size=mission.sitesize[0], poi=[], robots=[])
    cp = Poi(name="cp{0}".format(base.name), loc=base.center, mediums=[0,1],typepoi="transition")
    base.poi.append(cp)
    mission.sites.append(base)

    # Pools Initialization
    for site in range(1,nbs):
        newcenter = (round(uniform(-mission.arenasize //2,mission.arenasize //2),1), round(uniform(-mission.arenasize //2,mission.arenasize //2),1), 0)
        s = mission.sites[0]
        while closeness(v1=newcenter, v2=s.center, thresh=s.size):
            for s in mission.sites:
                newcenter = (round(uniform(-mission.arenasize //2,mission.arenasize //2),1), round(uniform(-mission.arenasize //2,mission.arenasize //2),1), 0)

        site = Site(name="site{0}".format(site),center=newcenter, size=randrange(mission.sitesize[0],mission.sitesize[1],10), poi=[], robots=[])
        cps = Poi(name="cp{0}".format(site.name), loc=(site.center[0],site.center[1],1), mediums=[1],typepoi="survey")
        site.poi.append(cps)
        mission.sites.append(site)

    pass

def Random_poi(mission=Mission(sites=[],robots=[])):
    """ Create a number of points of interest (POI) for all sites in a mission scenario """
    tot_nb_pp=0
    tot_nb_sp=0

    for site in mission.sites[1::]:
        nb_pp, nb_sp = site.size//10, site.size//7
        depth=site.size//10
        for _ in range(nb_pp):
            poiloc=(round(uniform(site.center[0]-site.size//2, site.center[0]+site.size//2),1), \
                    round(uniform(site.center[1]-site.size//2, site.center[1]+site.size//2),1),\
                    -depth)
            pp = Poi(name="pp{0}".format(tot_nb_pp+1), loc=poiloc, mediums=[-1],typepoi="sample")
            site.poi.append(pp)
            tot_nb_pp+=1
        
        for _ in range(nb_sp):
            poiloc=(round(uniform(site.center[0]-site.size//2, site.center[0]+site.size//2),1), \
                    round(uniform(site.center[1]-site.size//2, site.center[1]+site.size//2),1),\
                    0)
            sp = Poi(name="sp{0}".format(tot_nb_sp+1), loc=poiloc, mediums=[-1,1],typepoi="transition")
            site.poi.append(sp)
            tot_nb_sp+=1

        
    pass

def Random_Robots(mission=Mission(sites=[],robots=[]),nbr = int()):
    """ Create a number of robots for all sites and initialize their position in a mission scenario """

    for r in range(len(mission.robots),nbr):

        if len(mission.sites) > 1:
            md =0

        else:
            md = randrange(-1,2,2)

        rob = Robot(name="robot{0}".format(r),loc=mission.sites[0].poi[0].loc,medium=md,poi=mission.sites[0].poi[0].name,site=mission.sites[0].name, energy=10000)

        mission.robots.append(rob)
    
    pass


##################### FUNCTIONS FOR ESTIMATION OF SITES COST ########################

def CostEstimOftour(poi=[], waterspeed=0.5, airspeed=1.5, assesscost=30, explorecost=15, switchcost=10):
    """ 
    Comprehensive cost estimation function for clustering (Step 1) 
    - Includes all execution costs for site processing 
    - Penalizes transitions between different mediums
    """

    num_poi = len(poi)
    distance_matrix = np.zeros((num_poi, num_poi))

    for i in range(num_poi):
        for j in range(num_poi):
            if i != j:
                speed = waterspeed if poi[i].typepoi == 'sample' else airspeed
                base_cost = distance(poi[i].loc, poi[j].loc) / speed
                
                # If switching from water to air or vice versa, add penalty
                if poi[i].typepoi != poi[j].typepoi:
                    base_cost += switchcost  

                distance_matrix[i][j] = base_cost

    _, tsp_cost = solve_tsp_dynamic_programming(distance_matrix)

    # Include execution times as part of the cost estimation
    total_cost = tsp_cost + sum([assesscost if p.typepoi == 'sample' else explorecost for p in poi])

    return tsp_cost, total_cost

def Clustersite(poi=[], nb_cluster=int, weightslist=None):
    """ Cluster the POI into 'nb_cluster' number of clusters"""

    positions = np.array([vp.loc for vp in poi])
    # Implementing Kmeans method 
    kmeans = KMeans(n_clusters=nb_cluster, random_state=0, n_init=10).fit(X=positions,sample_weight=weightslist)

    # Labels list for distribution of vp
    labels = []

    # Store as clusters (2D array of viewpoints)
    clusters = [[] for _ in range(nb_cluster)]

    for vp, label in enumerate(kmeans.labels_):
        clusters[label].append(poi[vp])
        labels.append(label)

    return clusters


# def compute_scaling_factors(num_samples, num_transitions, r):
#     """
#     Computes k, beta, and gamma dynamically based on site complexity
#     and robot coalition size.
#     """
#     # Site complexity scaling factor
#     k = 1 + (num_samples / (num_samples + num_transitions + 2))  # Adapts with site difficulty
    
#     # Diminishing return factor for additional robots (logarithmic scaling)
#     beta = 0.9 + 0.2 * np.log(1 + num_samples)  # More tasks → slower diminishing returns
    
#     # Relay efficiency factor, increases if many transition points (relay-heavy site)
#     gamma = 0.85 + 0.15 * (num_transitions / (num_samples + num_transitions + 1))  
    
#     return k, beta, gamma

def compute_scaling_factors(num_samples, num_transitions, num_robots):
    """
    Compute scaling factors based on mission constraints.
    
    - gamma: relay efficiency factor (reduced when more transitions are needed).
    - beta: parallel execution scaling factor (limits efficiency of adding robots).
    - k: diminishing returns scaling factor (scales relative to site complexity).
    """
    
    # ✅ Compute relay efficiency (gamma)
    gamma = 1 - (num_transitions / (num_samples + num_transitions + 1))

    # ✅ Compute parallel execution scaling (beta)
    beta = num_samples / (num_samples + num_robots)

    # ✅ Compute diminishing returns factor (k)
    k = num_samples / (num_samples + num_transitions + 1)
    
    return k, beta, gamma
    
def get_weights_of_sites(sites=[], robots=[], listrused=[]):
    """ 
    Heuristic for estimating site resolution cost:
    - Models real execution behavior.
    - Ensures parallel execution scaling is balanced.
    - Adjusts for diminishing returns and navigation impact.
    """
    
    if not listrused:
        listrused = getlistusedRgroups(robots)
    
    weights_sites_list = []
    costs_of_all_sites = []

    for s in sites:
        cost_of_site = []
        num_samples = sum(1 for p in s.poi if p.typepoi == "sample")  # Number of sampling points
        num_transitions = sum(1 for p in s.poi if p.typepoi == "transition")  # Number of transition points

        # 🔥 Compute total site travel distance (for intra-site navigation cost)
        total_site_distance, sum_site_distances = CostEstimOftour(s.poi)  

        # 🔥 Compute adaptive scaling factors based on site properties
        k, beta, gamma = compute_scaling_factors(
            num_samples, num_transitions, len(listrused))

        for r in listrused:  # ✅ At least 2 robots required
            maxclust = min(r, len(s.poi) - 1)
            sample_clusters = []
            relay_clusters = []
            transition_clusters = []
            
            clusters = Clustersite(poi=s.poi[1:], nb_cluster=maxclust)
            for cluster in clusters:
                cluster.insert(0, s.poi[0])

                if cluster[-1].typepoi == "sample":
                    last_sample_poi = cluster[-1]
                    
                    # Find the closest transition POI
                    transition_pois = [p for p in s.poi if p.typepoi == "transition"]
                    if transition_pois:
                        closest_transition_poi = min(transition_pois, key=lambda p: distance(last_sample_poi.loc, p.loc))
                        cluster.insert(1, closest_transition_poi)

                # **Separate Sample, Relay, and Transition Clusters**
                if any(p.typepoi == "sample" for p in cluster):
                    sample_clusters.append(cluster)
                elif any(p.typepoi == "transition" for p in cluster):
                    relay_clusters.append(cluster)
                else:
                    transition_clusters.append(cluster)

            # **Estimate Cost for Sampling, Relay, and Transition Separately**
            sample_costs = [CostEstimOftour(cluster)[1] for cluster in sample_clusters]
            relay_costs = [CostEstimOftour(cluster)[1] for cluster in relay_clusters]
            transition_costs = [CostEstimOftour(cluster)[1] for cluster in transition_clusters]

            # ✅ **Parallel Execution Simulation**
            num_sample_robots = max(1, r - 1)  # At least one sampling robot
            relay_cost = sum(relay_costs) * gamma  # 🔥 Adjusted relay efficiency
            sample_cost = (sum(sample_costs) / num_sample_robots) * beta  # 🔥 Adjusted parallel workload

            estimated_cost = max(relay_cost, sample_cost) + sum(transition_costs)  # 🔥 Ensuring correct execution behavior

            # ✅ **Apply Heuristic Scaling**
            adjusted_cost = estimated_cost * (1 / (1 + k * (r - 2) ** beta)) + (total_site_distance / r)

            # ✅ **Ensure Cost Never Increases with More Robots**
            if len(cost_of_site) > 0 and adjusted_cost > cost_of_site[-1]:
                adjusted_cost = cost_of_site[-1]  # Force non-increasing cost behavior

            cost_of_site.append(round(adjusted_cost, 2))

        costs_of_all_sites.append(cost_of_site)
        weights_sites_list.append(sum(cost_of_site))

    return weights_sites_list, costs_of_all_sites




######################### GENERATING PDDL SCRIPTS ########################


def generateTMProblemPDDL(mission=Mission(sites=[],robots=[]), num=str()):
    """ Generate the Trans-Media problem as a PDDL script from mission scenario dataclass """

    # # # Writing the PDDL file

    # First lines for pddl files
    Debutlines = ['; mission = {0}'.format(mission),'(define (problem TMMSproblem)', '(:domain MMdomainextended)']


    # Objects definitions
    objectlines = ['(:objects']
    Sites, Poi, Robots = "", "", ""

    for r in mission.robots:
        Robots += "{0} ".format(r.name)
    Robots += "- robot "
    objectlines.append('')
    objectlines.append(Robots)

    for s in mission.sites:
        Sites += "{0} ".format(s.name)
        for p in s.poi:
            Poi += "{0} ".format(p.name)

    Sites += "- site "
    objectlines.append('')
    objectlines.append(Sites)
    Poi += "- pointofinterest "    
    objectlines.append('')
    objectlines.append(Poi)
    objectlines.append('')
    objectlines.append(')')

    # initial state definitions

    initstatelines = ['(:init \n','; Robot states \n']

    for r in mission.robots:
        initstatelines.append('(at {0} {1})'.format(r.name,r.poi))
        initstatelines.append('(at_site {0} {1})'.format(r.name,r.site))
        if r.medium==1:
            initstatelines.append('(airconf {0})'.format(r.name))
        elif r.medium == -1:
            initstatelines.append('(waterconf {0})'.format(r.name))
        elif r.medium == 0:
            initstatelines.append('(groundconf {0})'.format(r.name))

        initstatelines.append('(available {0})'.format(r.name))
        initstatelines.append('(canswitch {0})'.format(r.name))
        initstatelines.append('(canrelay {0})'.format(r.name))
        initstatelines.append('(cansample {0})'.format(r.name))
        initstatelines.append('')

    initstatelines.append('')
    
    # sites location
    initstatelines.append('; Poi/Sites states')

    for s in mission.sites:
        initstatelines.append('')
        initstatelines.append(';{0}'.format(s.name))

        for p in s.poi:

            if p.typepoi == "transition":
                initstatelines.append('(transition_poi {0})'.format(p.name))
                initstatelines.append('(isswitchable {0})'.format(p.name))
                initstatelines.append('(isrelay {0})'.format(p.name))
            elif p.typepoi == "survey":
                initstatelines.append('(survey_poi {0})'.format(p.name))
            else:
                initstatelines.append('(sample_poi {0})'.format(p.name))

            if 0 in p.mediums: 
                initstatelines.append('(ground {0})'.format(p.name))
            if -1 in p.mediums:
                initstatelines.append('(water {0})'.format(p.name))
            if 1 in p.mediums:
                initstatelines.append('(air {0})'.format(p.name))

            initstatelines.append('(partofsite {0} {1})'.format(p.name, s.name))
            initstatelines.append('')

    initstatelines.append('')
    # Functions robot

    initstatelines.append(';Functions')
    initstatelines.append('')

    for r in mission.robots:
        initstatelines.append(';{0}'.format(r.name))
        initstatelines.append(';Speeds')
        initstatelines.append('(= (speedair {0}) 1.5)'.format(r.name))
        initstatelines.append('(= (speedwater {0}) 0.5)'.format(r.name))
        initstatelines.append(';Energy')
        initstatelines.append('(= (energy {0}) 1000000000)'.format(r.name))
        initstatelines.append('(= (recharge_rate {0}) 10)'.format(r.name))
        initstatelines.append(';Cost')
        initstatelines.append('(= (assesscost {0}) 0.02)'.format(r.name))
        initstatelines.append('(= (partassesscost {0}) 0.01)'.format(r.name))
        initstatelines.append('(= (observecost {0}) 0.01)'.format(r.name))
        initstatelines.append(';Consumption')
        initstatelines.append('(= (maintainconsumption_water {0}) 0.02)'.format(r.name))
        initstatelines.append('(= (maintainconsumption_air {0}) 0.08)'.format(r.name))
        initstatelines.append('(= (navconsumption_water {0}) 0.5)'.format(r.name))
        initstatelines.append('(= (navconsumption_air {0}) 0.8)'.format(r.name))
        initstatelines.append(';Switch')
        initstatelines.append('(= (switchduration_airwater {0}) 5)'.format(r.name))
        initstatelines.append('(= (switchcost_airwater {0}) 20)'.format(r.name))
        initstatelines.append('(= (takeoffduration_groundair {0}) 4)'.format(r.name))
        initstatelines.append('(= (takeoffost_groundair {0}) 10)'.format(r.name))
        initstatelines.append('(= (landingduration_airground {0}) 3)'.format(r.name))
        initstatelines.append('(= (landingcost_airground {0}) 5)'.format(r.name))
        initstatelines.append('(= (switchduration_waterair {0}) 8)'.format(r.name))
        initstatelines.append('(= (switchcost_waterair {0}) 30)'.format(r.name))
        initstatelines.append('')

    initstatelines.append(';DistancesPoiSites')
    initstatelines.append('')

    for s in mission.sites:
        initstatelines.append(';{0}'.format(s.name))
        initstatelines.append('')
        initstatelines.append('(= (site_size {0}) {1})'.format(s.name, s.size))
        initstatelines.append('')

        for onepoi in s.poi:
            for twos in mission.sites:
                for twopoi in twos.poi:
                    
                    if onepoi.name != twopoi.name:
                        
                        if onepoi.typepoi =="transition" and (twopoi.typepoi == "survey" or twopoi.typepoi =="transition" or (twopoi.typepoi == "sample" and twos.name == s.name)):
                            initstatelines.append('(= (distance {0} {1}) {2})'.format(onepoi.name,twopoi.name, distance(onepoi.loc,twopoi.loc)))
                        elif onepoi.typepoi =="survey" and (twopoi.typepoi == "survey" or twopoi.typepoi =="transition"):
                            initstatelines.append('(= (distance {0} {1}) {2})'.format(onepoi.name,twopoi.name, distance(onepoi.loc,twopoi.loc)))
                        elif onepoi.typepoi =="sample" and ((twopoi.typepoi == "sample" and twos.name == s.name) or (twopoi.typepoi =="transition" and twos.name == s.name)):
                            initstatelines.append('(= (distance {0} {1}) {2})'.format(onepoi.name,twopoi.name, distance(onepoi.loc,twopoi.loc)))

            initstatelines.append('')

    initstatelines.append('')
    initstatelines.append(')')

    # goal definitions
    goallines = ['(:goal (and']

    for s in mission.sites:
        for p in s.poi:
            # if p.name.startswith("sp"):
            #     goallines.append('(part_assessed {0})'.format(p.name))
            if p.typepoi == "sample":
                goallines.append('(sampled {0})'.format(p.name))

    goallines.append(')')
    goallines.append(')')
    # writing the file

    totallines = Debutlines + objectlines + initstatelines + goallines
    totallines.append(')')

    with open('{0}.pddl'.format(num), 'w') as f:
        for line in totallines:
            f.write(line)
            f.write('\n')


def GenerateAdaptedProblem(mission, previous_problem, robot_state, next_site, goaltype):
    """ 
    Generate a sub-problem PDDL file for each robot following the mission scenario.
    
    This function modifies an existing PDDL problem file by:
    - Keeping only the robots in `robot_state`
    - Updating `at`, `at_site`, and `conf` predicates based on the `current_site`
    - Modifying the `goal` to ensure the robots sample `next_site`
    - Writing the new problem to `/tmp/subproblems/`
    """
    
    # Get unique **starting sites** for all robots
    current_sites = {robot_state[r]["site"] for r in robot_state}
    

    # Retrieve site objects from mission based on names
    current_site_objs = [site for site in mission.sites if site.name in current_sites]
    next_site_obj = [site for site in mission.sites if site.name == next_site]
    next_site = next_site_obj[0]

    # print("generate adapted problem",current_sites, "future name : ",f"subp_{'_'.join(sorted(current_sites))}_to_{next_site.name}_{goaltype}")

    if "pddl" not in previous_problem:
        previous_problem += ".pddl"

    with open(previous_problem, 'r') as file:
        lines = file.readlines()

    new_lines = []
    goal_added = False  # Flag to avoid adding `goal` multiple times

    for line in lines:
        line = line.strip()

        # Modify the problem definition line (Ensure domain remains intact)
        if "define" in line:
            problem_name = f"subp_{'_'.join(sorted(current_sites))}_to_{next_site.name}"
            new_lines.append(f"(define (problem {problem_name})\n")
            continue 

        # Handle point of interest definition (include POIs from both current & next sites)
        elif "- pointofinterest" in line:
            tmpline = line.split(" ")
            filtered_pois = [
                exp for exp in tmpline
                if any(exp == poi.name for site in current_site_objs + [next_site] for poi in site.poi)
            ]
            new_lines.append(" ".join(filtered_pois) + " - pointofinterest\n")
            continue
        
        # Handle site definition
        elif "- site" in line:
            tmpline = line.split(" ")
            filtered_sites = [exp for exp in tmpline if exp in current_sites or exp == next_site.name]
            new_lines.append(" ".join(filtered_sites) + " - site\n")
            continue

        # Handle robot definition
        elif "- robot" in line:
            tmpline = line.split(" ")
            filtered_robots = [exp for exp in tmpline if exp in robot_state.keys()]
            new_lines.append(" ".join(filtered_robots) + " - robot\n")
            continue

        # Process only relevant robot-related lines
        elif "robot" in line:
            if any(robot in line for robot in robot_state.keys()):
                modified = False  # Flag to track if we modified the line
                
                for robot in robot_state.keys():
                    if 'at {} '.format(robot) in line:
                        new_lines.append(f"( at {robot} {robot_state[robot]['position']} )\n")
                        modified = True
                    elif 'at_site {} '.format(robot) in line:
                        new_lines.append(f"( at_site {robot} {robot_state[robot]['site']} )\n")
                        modified = True
                    elif 'conf {} '.format(robot) in line:
                        new_lines.append(f"( {robot_state[robot]['conf']} {robot} )\n")
                        modified = True
                
                if not modified:
                    new_lines.append(line + "\n")  # If not modified, keep the original line
            else:
                continue
            
        # Filter out POI distance definitions that don't involve the current or next site
        elif "sp" in line or "pp" in line or "cp" in line:
            tmpline = line.split(" ")
            poi_in_line = [exp for exp in tmpline if "sp" in exp or "pp" in exp or "cp" in exp]
            
            # Keep only lines related to the current or next site
            if all(
                    any(exp == poi.name for poi in next_site.poi)  # At least one POI belongs to next_site
                    or any(exp == poi.name for site in current_site_objs for poi in site.poi)  # OR at least one belongs to a current site
                    for exp in poi_in_line
                ):
                new_lines.append(line + "\n")   
            continue  # Skip lines that reference POIs from other sites
        
        elif "site_size" in line:
            if any(site in line for site in current_sites) or next_site.name in line:
                new_lines.append(line + "\n")
            continue  # Remove other site size definitions

        # Modify the goal section
        elif 'goal' in line and not goal_added:
            new_lines.append("(:goal\n")  # Define goal block
            new_lines.append("( and\n")
            if goaltype == "goto":      
                if next_site.name == "base":
                    for robot in robot_state.keys():
                        new_lines.append("( groundconf {0})\n".format(robot))
                else:
                    for robot in robot_state.keys():
                        new_lines.append(f"( at_site {robot} {next_site.name} )\n")
            elif goaltype == "solving":
                for p in next_site.poi:
                    if p.typepoi == "sample":
                        new_lines.append("(sampled {0})\n".format(p.name))
            new_lines.append(")\n")  # Close the `and` block
            new_lines.append(")\n")  # Close `goal`
            new_lines.append(")\n") # Close `define`
            goal_added = True  # Ensure we only modify goal once
            break

        else:
            new_lines.append(line + "\n")  # Keep all other lines

    # Create the output directory if it doesn't exist
    output_dir = "/tmp/plan_output/subproblems/"

    # Define the new problem file path
    updated_path = os.path.join(output_dir, f"subp_{'_'.join(sorted(current_sites))}_to_{next_site.name}_{goaltype}")
    # print("generate adapted problem" "updated path : ",updated_path)
    # Write the updated problem file
    with open(updated_path +".pddl", 'w') as file:
        file.writelines(new_lines)

    return updated_path

def getdataclassfrompddl(file):
    """ Create data class from PDDL problem script """
    mission = Mission(sites=[],robots=[])
    with open(file, 'r') as f:
            contents = f.readlines()
            if contents[0].startswith("; mission = "):
                mission = eval(contents[0].partition("= ")[2])
    return mission

import json
from pathlib import Path
from typing import Dict
from dataclasses import asdict

def recover_mission_from_json(json_file: Path) -> Mission:
    """
    Reads a world_info.json file and reconstructs the Mission data class.
    
    :param json_file: Path to the JSON file containing world information.
    :return: Mission object reconstructed from the JSON data.
    """
    # Load the JSON data
    with open(json_file, 'r') as f:
        data = json.load(f)

    # Create a dictionary to map POI IDs to Poi objects
    points: Dict[str, Poi] = {}
    for point in data["points"]:
        # Infer mediums based on the POI type
        mediums = {
            "transition": [-1, 1],
            "survey": [1],
            "sample": [-1]
        }.get(point["type"], [])  # Default to empty if unknown

        # Create a Poi object and store it in the map
        points[point["id"]] = Poi(
            mediums=mediums,
            name=point["id"],
            loc=tuple(point["coordinates"]),
            typepoi=point["type"]
        )

    # Parse sites and associate POIs
    sites = []
    for site in data["sites"]:
        site_pois = [points[poi_id] for poi_id in site["points"]]

        # Compute center coordinates
        if site_pois:
            x_center = sum(poi.loc[0] for poi in site_pois) / len(site_pois)
            y_center = sum(poi.loc[1] for poi in site_pois) / len(site_pois)
            z_center = sum(poi.loc[2] for poi in site_pois) / len(site_pois)
            center = (x_center, y_center, z_center)
        else:
            center = (0, 0, 0)

        sites.append(Site(
            poi=site_pois,
            name=site["id"],
            center=center,
            size=site["size"]
        ))

    # Parse robots if they exist in the JSON file
    robots = []
    if "robots" in data:
        for robot in data["robots"]:
            robots.append(Robot(
                name=robot["name"],
                loc=tuple(robot["loc"]),
                medium=robot["medium"],
                histo=robot.get("histo", []),
                currentpathcost=robot.get("currentpathcost", 0),
                poi=robot["poi"],
                site=robot["site"],
                energy=robot["energy"]
            ))

    # Construct and return the Mission object
    mission = Mission(
        sites=sites,
        robots=robots,
        objective=data.get("objective", "explore"),
        arenasize=data.get("arenasize", 1000),
        sitesize=tuple(data.get("sitesize", [10, 50]))
    )

    return mission


##################### CORE ALGORITHM TO GET BEST ASSIGNEMENT ########################
def alternate_from_both_sides(optimal_order):
    """
    Reorders sites in a zigzag pattern:
    - First site from the start
    - Then from the end
    - Alternates between start and end

    Example:
    Input:  ['site1', 'site2', 'site4', 'site6', 'site7']
    Output: ['site1', 'site4', 'site6', 'site7', 'site2']
    """
    left = 0
    right = len(optimal_order) - 1
    zigzag_order = []

    while left <= right:
        if left == right:
            zigzag_order.append(optimal_order[left])
        else:
            zigzag_order.append(optimal_order[left])
            zigzag_order.append(optimal_order[right])

        left += 1
        right -= 1

    return zigzag_order

from scipy.spatial.distance import cdist
import numpy as np
from sklearn.cluster import KMeans

def Weightned_Cluster(mission, numb_clt=2):
    """ Improved clustering with both site location and estimated processing cost.
    Now ensures the sites in each cluster follow an optimal order based on TSP
    and are then allocated from both sides alternately.
    """

    # Extract site centers and weights
    list_of_sites_center = np.array([s.poi[0].loc for s in mission.sites[1:] if s.poi])
    weights_sites_list, _ = get_weights_of_sites(sites=mission.sites[1:], robots=mission.robots)
    
    # Normalize weights to balance location importance
    weights_sites_list = np.array(weights_sites_list) / np.max(weights_sites_list)
    
    # Augment the site coordinates with weight information
    augmented_sites = np.column_stack((list_of_sites_center, weights_sites_list))
    
    # Apply KMeans clustering with weighted sites
    kmeans = KMeans(n_clusters=numb_clt, random_state=42, n_init=10).fit(augmented_sites)
    
    # Initialize clusters
    clusters_list = [Cluster_class(name=f"cluster{cl}") for cl in range(numb_clt)]
    for idx, label in enumerate(kmeans.labels_):
        clusters_list[label].sites.append(mission.sites[idx+1])

    # ✅ **Rearrange sites within each cluster based on the optimal path**
    base_site = mission.sites[0]  # The base site

    for cluster in clusters_list:
        if len(cluster.sites) > 1:
            # Compute the optimal order of sites in the cluster
            optimal_order_names = find_optimal_path(cluster.sites, base_site)

            # ✅ **Reorder sites alternately from both sides**
            zigzag_order_names = alternate_from_both_sides(optimal_order_names)

            # Reorder the cluster sites to follow this allocation-friendly path
            cluster.sites.sort(key=lambda site: zigzag_order_names.index(site.name))

    return clusters_list


def calculate_path_cost_estim(
    scenario: ScenarioAttribution, 
    Robots: list, 
    TaskList: list,  
    listusedRgroups: list, 
    base: Site, 
    level: str = "site"
):
    """
    **Fixed Scenario Cost Estimation**
    - Ensures no robots are left idle if they can contribute.
    - Assigns robots based on proximity to minimize movement cost.
    - Synchronization adjusted to prioritize execution over waiting.
    """

    idx = 0
    available_robots = Robots[:]  # Copy list of robots

    for assignment, task in zip(scenario.scenario, TaskList):
        available_robots = Robots[:]  # Reset available robots per site
        # if level =="cluster":
        #     target_location = min(task.sites, key=lambda s: distance(base.center, s.center)).center
        # else:
        #     target_location = task.center
        target_location = task.center
        costaction = scenario.costmatrix[idx][listusedRgroups.index(assignment)]

        while len(task.robots) < assignment:        
            assigncost = []
            for r in available_robots:
                
                # ✅ **Assign robots based on travel distance to the site**
                travel_time = round(distance(r.loc, target_location) / 1.5, 2)
                accumulated_cost = costaction + r.currentpathcost + travel_time
                assigncost.append(accumulated_cost)

            # ✅ **Choose the best robot (min cost robot)**
            best_robot_idx = assigncost.index(min(assigncost))
            best_robot = available_robots[best_robot_idx]

            # ✅ Assign robot and track its arrival time
            task.robots.append(best_robot)
            best_robot.currentpathcost += assigncost[best_robot_idx]
            best_robot.loc = target_location
            best_robot.histo.append(task.name)

            available_robots.remove(available_robots[best_robot_idx])
            # # Sync only those robots whose arrival times are far off
            # site_sync_time = max(avg_site_cost, max_site_cost * 0.8)

            for r in task.robots:
                r.currentpathcost = max([r.currentpathcost for r in task.robots])

        idx += 1  

    for r in Robots:
        r.currentpathcost += round(distance(r.loc, base.center), 2) 
    # ✅ **Final mission cost based on the highest accumulated robot path**
    scenario.totalcost = max([r.currentpathcost for r in Robots])
    # ✅ **Reset robot states after simulation**
    for r in Robots:
        r.currentpathcost = 0
        r.histo = []
        r.loc = base.center

    for t in TaskList:
        t.robots = []
        t.total_completion_time = 0
        if level == "cluster":
            for s in t.sites:
                s.robots = []
                s.total_completion_time = 0

    return scenario

def getbestassignforclusterandR(num_robots=0, costsites=np.array([]),listusedRgroups=[], cluster=[], base=Site(poi=[], robots=[]),level="site",Robots=[]):
    """ Go through all possible assignement scenario for a given number of sites and robots and select the optimal allocation """    
    
    # Example data
    cost_matrix = costsites
    num_sites = len(cost_matrix)
    # Generate all possible robot assignments to sites
    possible_assignments = list(product(listusedRgroups, repeat=num_sites))
    # print("in get best assign cluster and R",num_robots, len(possible_assignments))
    # Calculate and store the cost for each generated scenario
    scenario_costs_dict = {}
    scenariosdict= {}
    for assignment in possible_assignments:
        
        sce = ScenarioAttribution(clusterinscenario=cluster,scenario=assignment, numrobots=num_robots,totalcost=0.0,costmatrix=cost_matrix)
        
        scenariosdict[assignment] = calculate_path_cost_estim(scenario=sce,Robots=Robots[:num_robots],TaskList=cluster,listusedRgroups=listusedRgroups,base=base,level=level)
        scenario_costs_dict[assignment] = sce.totalcost
        # print("scenario and cost",scenariosdict[assignment].scenario, scenario_costs_dict[assignment])
    
    min_cost = min(scenario_costs_dict.values())
    optimal_cost = max((k for k, v in scenario_costs_dict.items() if v == min_cost), key=sum)
    optimal_scenario = scenariosdict[optimal_cost]
    # print("LE OPTIMAL", optimal_scenario.scenario, optimal_scenario.totalcost, [s.name for s in cluster], num_robots)
    return optimal_scenario
           
def Assign_robots_to_scenario(
    scenario, Robots=[], Clusters=[], costsites=[], listusedRgroups=[], base=Site(poi=[], robots=[])
):
    """ 
    Assign robots optimally to a given scenario of cluster and site allocations. 
    - Ensures efficient workload distribution.
    - Prioritizes minimizing makespan.
    """

    idx = 0  # Keeps track of scenario step

    for assignment, cluster in zip(scenario.scenario, Clusters):
        cluster.centroid_of_sites_2d  # Ensure cluster centroid is up-to-date
        available_robots = Robots[:]  # Copy robot list (to track availability)
        
        # ✅ Retrieve the **precomputed** cost estimation
        costaction = scenario.costmatrix[idx][listusedRgroups.index(assignment)]

        # ✅ Assign robots **to clusters first**
        while len(cluster.robots) < assignment:
            
            assigncost = [
                costaction + r.currentpathcost + distance(r.loc, cluster.sites[0].center) / 1.5
                for r in available_robots
            ]

            # ✅ Pick the **best** robot (one with **minimum cost accumulation**)
            best_robot_idx = assigncost.index(min(assigncost))
            best_robot = available_robots[best_robot_idx]

            # ✅ Assign robot to the cluster
            cluster.robots.append(available_robots[best_robot_idx])
            available_robots[best_robot_idx].currentpathcost += min(assigncost)  # 🔥 Accumulate **real execution cost**
            available_robots[best_robot_idx].loc = cluster.sites[0].center  # Move robot to first site in cluster

            # ✅ Remove assigned robot from available list
            available_robots.pop(best_robot_idx)

        

        # ✅ **Second Phase: Assign Robots to Individual Sites**
        # print("best scenario for cluster ", cluster.name, " with ", [s.name for s in cluster.sites], "  scenario: ",cluster.bestscenario[str(assignment)])
        for r in cluster.robots:
            r.currentpathcost = 0
            r.histo = []
            r.loc = base.center
        # print("clusterobots ", [(r.name) for r in cluster.robots], [(s.robots) for s in cluster.sites])
        _, clustersitescost = get_weights_of_sites(sites=cluster.sites, robots=cluster.robots)
        # print(clustersitescost)
        cost_matrix = clustersitescost

        # print(cost_matrix)
        idxsite = 0
        for sassign, site in zip(cluster.bestscenario[str(assignment)], cluster.sites):
            
            available_robots_in_site = cluster.robots[:]  # Reuse robots assigned to the cluster
            costactionsite = cost_matrix[idxsite][listusedRgroups.index(sassign)]
            target_location = site.center
            # print(sassign, site.name, listusedRgroups, costactionsite)
            while len(site.robots) < sassign:
                
                siteassigncost = []
                for r in available_robots_in_site:
                    
                    # ✅ **Assign robots based on travel distance to the site**
                    travel_time = round(distance(r.loc, target_location) / 1.5, 2)
                    accumulated_cost = costactionsite + r.currentpathcost + travel_time
                    # print(r.name, r.loc, r.site, site.name, site.center,distance(r.loc, target_location), accumulated_cost, ": ", r.currentpathcost, travel_time, costactionsite)
                    siteassigncost.append(accumulated_cost)

                # print("costactionsite,travel_time,accumulated_cost,siteassigncost,sassign, site      ",costactionsite,travel_time,accumulated_cost,siteassigncost,sassign, site.name)
                # ✅ Pick the **best** robot for the site
                best_robot_idx = siteassigncost.index(min(siteassigncost))
                best_robot = available_robots_in_site[best_robot_idx]

                # print(siteassigncost, best_robot.name)
                # ✅ Assign robot **and update movement cost**
                site.robots.append(best_robot)
                best_robot.currentpathcost += min(siteassigncost)
                best_robot.loc = site.center
                best_robot.histo.append(site.name)
                best_robot.site = site.name

                # ✅ Remove from available site-level list
                available_robots_in_site.remove(available_robots_in_site[best_robot_idx])

            for r in site.robots:
                r.currentpathcost = max([r.currentpathcost for r in site.robots])

            idxsite += 1
        idx += 1  # Move to next step

    return scenario  # ✅ Return the scenario with assigned robots





# for assignment, task in zip(scenario.scenario, TaskList):
#         available_robots = Robots[:]  # Reset available robots per site
#         # if level =="cluster":
#         #     target_location = min(task.sites, key=lambda s: distance(base.center, s.center)).center
#         # else:
#         #     target_location = task.center
#         target_location = task.center
#         costaction = scenario.costmatrix[idx][listusedRgroups.index(assignment)]

#         while len(task.robots) < assignment:        
            # assigncost = []
            # for r in available_robots:
                
            #     # ✅ **Assign robots based on travel distance to the site**
            #     travel_time = round(distance(r.loc, target_location) / 1.5, 2)
            #     accumulated_cost = costaction + r.currentpathcost + travel_time
            #     assigncost.append(accumulated_cost)

#             # ✅ **Choose the best robot (min cost robot)**
#             best_robot_idx = assigncost.index(min(assigncost))
#             best_robot = available_robots[best_robot_idx]

#             # ✅ Assign robot and track its arrival time
#             task.robots.append(best_robot)
#             best_robot.currentpathcost += assigncost[best_robot_idx]
#             best_robot.loc = target_location
#             best_robot.histo.append(task.name)

#             available_robots.remove(available_robots[best_robot_idx])
#             # # Sync only those robots whose arrival times are far off
#             # site_sync_time = max(avg_site_cost, max_site_cost * 0.8)

#             for r in task.robots:
#                 r.currentpathcost = max([r.currentpathcost for r in task.robots])

#         idx += 1  

#     for r in Robots:
#         r.currentpathcost += round(distance(r.loc, base.center), 2) 
















def getbestassignfrommission(mission=Mission(sites=[], robots=[]), minnbofcluster=2, get_all_time=True):
    """ Main algorithm, Clusterize a number of sites into equal weight clusters and identify the best scenario of pre-allocation possible for a number of robots and number of sites  """ 

    t_alloc_full_0 = time.perf_counter()
    
    t_clustering_0 = time.perf_counter()  
    listusedRgroups = getlistusedRgroups(robots=mission.robots)

    clusterslist=[]
    for numcl in range(minnbofcluster,len(mission.sites)//2 +1):
        clt =Weightned_Cluster(mission=mission,numb_clt=numcl)
        # showclusters(mission, clt, mission.sites[0])
        clusterslist.append(clt)

    t_clustering_1 = time.perf_counter() 
    # We evaluate the best tour cost for every allocation possibility 
    all_scenario_for_clusters=[]
    all_scenario_for_clusters_costmatrix=[]
    
    # print("\n","BEFORE FIRST LOOP", "\n", "clusterslist",clusterslist, "\n")
    t_sce_sites_0 = time.perf_counter() 
    for XCcluster in clusterslist:
        scenarioXCclusterlist=[]
        scenarioXCclusterlist_costmatrix=[]
        # print("clusterisation in", len(XCcluster)," Clusters")
        for cluster in XCcluster:
            weirghtofsites, clustersitescost=get_weights_of_sites(sites=cluster.sites, robots=mission.robots,listrused=listusedRgroups)
            # print([s.name for s in cluster.sites], "\n",clustersitescost,"\n",weirghtofsites)
            scenarioclusterlist=[]
            scenarioclusterlist_costmatrix=[]

            for r in listusedRgroups:
                
                sce = getbestassignforclusterandR(num_robots=r, costsites=clustersitescost, listusedRgroups=getlistusedRgroups(mission.robots[:r]), cluster=cluster.sites, base=mission.sites[0], Robots=mission.robots, level="site")
                scenarioclusterlist.append(sce)
                scenarioclusterlist_costmatrix.append(sce.totalcost)
                cluster.bestscenario[str(r)]=sce.scenario
                # print("FINAL TOTAL COST", listusedRgroups[:(listusedRgroups.index(r)+1)], sce.totalcost, r, sce.scenario)

            scenarioXCclusterlist.append(scenarioclusterlist)
            scenarioXCclusterlist_costmatrix.append(scenarioclusterlist_costmatrix)
            

        all_scenario_for_clusters.append(scenarioXCclusterlist)
        all_scenario_for_clusters_costmatrix.append(scenarioXCclusterlist_costmatrix)
    t_sce_sites_1 = time.perf_counter()       
    # We find the best allocation for every XC clusters
    scenarioC=[]
    costC=[]
    # print("CLUSTER PART \n")

    t_sce_cluster_0 = time.perf_counter()
    for XCclustercost, CCluster in zip(all_scenario_for_clusters_costmatrix,clusterslist):
        # print("\n", len(XCclustercost), len(CCluster), XCclustercost, CCluster, "\n")
        sce = getbestassignforclusterandR(num_robots=listusedRgroups[-1], Robots=mission.robots, costsites=XCclustercost, listusedRgroups=listusedRgroups, cluster=CCluster, base=mission.sites[0], level="cluster")
        costC.append(sce.totalcost)
        scenarioC.append(sce)

    t_sce_cluster_1 = time.perf_counter()
    t_alloc_full_1 = time.perf_counter()

    # print("Possibles clustering repartition with best scenario :", [i.scenario for i in scenarioC], "with cost of :",  costC)
    # print(clusterslist)
    
    print("best scenario is:",scenarioC[costC.index(min(costC))].scenario, "with cost of :",  min(costC))    
    
    # print(all_scenario_for_clusters[scenarioC.index(scenarioC[costC.index(min(costC))])], "\n")
    clusters= scenarioC[costC.index(min(costC))].clusterinscenario
    
    t_assignrob_0 = time.perf_counter()
    # print("BAH VOYONS: ", scenarioC[costC.index(min(costC))])
    Assign_robots_to_scenario(scenario=scenarioC[costC.index(min(costC))], Robots=mission.robots,Clusters=clusters, listusedRgroups=listusedRgroups, base=mission.sites[0])
    t_assignrob_1 = time.perf_counter()

    allocationscenario = scenarioC[costC.index(min(costC))]


    t_alloc_full_1 = time.perf_counter()
    print("time to calculate the pre-alloc:", t_alloc_full_1-t_alloc_full_0,)
    print("time of the Clustering:", t_clustering_1-t_clustering_0)
    print("time for scenario allocation (sites scenario estim):", t_sce_sites_1-t_sce_sites_0)
    print("time for scenario allocation (cluster final scenario estim):", t_sce_cluster_1-t_sce_cluster_0)
    print("time to assign the robots:", t_assignrob_1-t_assignrob_0,"\n")

    return clusters, allocationscenario

def findpoint(name="", sites=[]):
    """  """

    for s in sites:
        for p in s.poi:
            if p.name == name:
                return p


def parse_plan_line(line):
    """
    Parse a line from the plan output to extract action details.
    Assumes line format is:
    'time: (action entity1 entity2 ...) [duration]'
    """
    try:
        # Extract the action details
        start_time = line.split(":")[0].strip()
        details = line.split("[")[0].split(":")[1].strip()
        duration = line.split("[")[1].split("]")[0].strip()
        action = details.split()[0]
        entities = details.split()[1:]
        
        return {
            "start_time": float(start_time),
            "action": action,
            "entities": entities,
            "duration": float(duration)
        }
    except Exception as e:
        print(f"Error parsing line: {line} - {e}")
        return None

def action_shape(action_name):
    """
    Determine the shape of the node based on the action name.
    """
    shapes = {
        "navigation": "o",  # Circle for navigation actions
        "switch": "s",      # Square for switch actions
        "translate": "d",   # Diamond for translate actions
        "assess": "^",      # Triangle for assess actions
    }
    for action_type, shape in shapes.items():
        if action_type in action_name:
            return shape
    return "o"

def read_plan_from_file(filename):
    """
    Read and parse the plan from a given text file.
    """
    actions = []
    with open(filename, "r") as file:
        for line in file:
            # Check if line contains an action (simple check for ": (" as indicator)
            if ": (" in line and "[" in line and "]" in line:
                action_details = parse_plan_line(line)
                actions.append(action_details)
    return actions

def visualize_plan(actions, dictshape, legend_entries):
    G = nx.DiGraph()
    positions = {}
    labels = {}
    tracks = {}
    # colors = {"robot0": "red", "robot1": "green"}
    colors = {"robot0": "red", "robot1": "green", "robot2": "blue", "robot3": "orange"}
    last_action_per_robot = {robot: None for robot in colors.keys()}
    translate_data_actions_by_site = {}
    assess_actions_by_site = {}
    survey_site_actions = {}  # Adjusted to track "survey_site_2r" actions by site
    
    start_times = sorted(set(action['start_time'] for action in actions))
    time_to_x = {time: index for index, time in enumerate(start_times, start=1)}
    
    # Add initial nodes for robot names
    for robot in colors.keys():
        robot_node_id = f"name_{robot}"  # A unique identifier for the robot name node
        tracks[robot] = len(tracks) + 1  # Assign track positions
        positions[robot_node_id] = (-0.5, tracks[robot])  # x=0 for all robot name nodes
        labels[robot_node_id] = robot
        G.add_node(robot_node_id, color=colors[robot], label_pos='left', shape=">")
        
    for i, action in enumerate(actions):
        involved_robots = [entity for entity in action['entities'] if entity.startswith('robot')]
        action_type = action['action'].strip("()")
        site = action['entities'][-1]  # Assuming the site or related entity is the last in the list

        for robot in involved_robots:
            pos_x = time_to_x[action['start_time']]
            pos_y = tracks.setdefault(robot, len(tracks) + 1)
            node_id = (i, robot)
            positions[node_id] = (pos_x, pos_y)
            labels[node_id] = f"{action_type}\n{' '.join(action['entities'][1:])}"
            updated_label = f"{labels[node_id]}\n(time={action['start_time']})"
            labels[node_id] = updated_label

            G.add_node(node_id, color=colors[robot], shape=dictshape.get(action_type))

            if last_action_per_robot[robot]:
                G.add_edge(last_action_per_robot[robot], node_id)

            last_action_per_robot[robot] = node_id

            if action_type == "translate_data":
                translate_data_actions_by_site.setdefault(site, []).append(node_id)
            if action_type == "assess":
                assess_actions_by_site.setdefault(site, []).append(node_id)
            if action_type == "survey_site_2r":
                survey_site_actions.setdefault(site, []).append(node_id)

    # Create edges for "assess" actions to "translate_data" actions
    for site, assess_node_ids in assess_actions_by_site.items():
        translate_node_ids = translate_data_actions_by_site.get(site, [])
        for assess_node_id in assess_node_ids:
            for translate_node_id in translate_node_ids:
                G.add_edge(translate_node_id, assess_node_id, color='black', style='dotted')

    # Link "survey_site_2r" actions across robots
    for site, node_ids in survey_site_actions.items():
        for src_node_id in node_ids:
            for dst_node_id in node_ids:
                if src_node_id != dst_node_id:  # Avoid linking a node to itself
                    G.add_edge(src_node_id, dst_node_id, color='blue', style='dashed')

    # Drawing section
    fig, ax = plt.subplots(figsize=(30, 15))
    for node, data in G.nodes(data=True):
        nx.draw_networkx_nodes(G, pos={node: positions[node]}, nodelist=[node], node_color=data['color'],node_shape=data["shape"], node_size=5000, ax=ax)
    nx.draw_networkx_labels(G, pos=positions, labels=labels, font_size=10, font_weight='bold', ax=ax)
    for u, v, data in G.edges(data=True):
        nx.draw_networkx_edges(G, pos=positions, edgelist=[(u, v)], edge_color=data.get('color', 'gray'), style=data.get('style', 'solid'), ax=ax)

    plt.xlabel("Sequential Steps")
    plt.xticks([i for i in range(0,len(start_times))])
    plt.ylabel("Robots")
    plt.yticks([tracks[robot] for robot in sorted(tracks)], sorted(tracks))
    plt.title("Robots Actions Plan Sequence with Links")
    # Create custom legend entries using Line2D
    legend_handles = [
    Line2D([], [], color=entry["color"], marker=entry["marker"], linestyle=entry["linestyle"], markersize=10, label=entry["label"]) for entry in legend_entries
    ]
    # Add legend to the plot with custom entries
    plt.legend(handles=legend_handles)
    plt.grid(False)
    ax.xaxis.grid(False)  # Optional: adjust based on preference
    plt.tight_layout()
    plt.show()


def merge_for_single_path(plans: List[Path], output: Path, domain, problem: Path):
    reader = PDDLReader()
    upf_pb = reader.parse_problem(domain, problem)
    actions: List[Tuple[Fraction, Any, Fraction]] = []
    for i,plan in enumerate(plans):
        upf_plan = reader.parse_plan(upf_pb, plan)
        end_plan = max((a[0] + a[2] for a in actions), default=0)
        for a in upf_plan.timed_actions:
            actions.append((a[0] + end_plan + Fraction(i,1000), a[1], a[2]))
    new_plan = TimeTriggeredPlan(actions)

    writer = PDDLWriter(upf_pb)
    writer.write_plan(new_plan, output)


def merge_for_multi_path(merged_plans: List[Path], output: Path, domain, problem: Path):
    reader = PDDLReader()
    upf_pb = reader.parse_problem(domain, problem)
    actions = []
    for plan in merged_plans:
        upf_plan = reader.parse_plan(upf_pb, plan)
        actions += upf_plan.timed_actions
    actions.sort(key=lambda x:x[0])
    new_plan = TimeTriggeredPlan(actions)

    writer = PDDLWriter(upf_pb)
    writer.write_plan(new_plan, output)

def convert_toSTN(plan: Path, output: Path, domain, problem: Path):
    reader = PDDLReader()
    upf_pb = reader.parse_problem(domain, problem)
    actions: List[Tuple[Fraction, Any, Fraction]] = []

    upf_plan = reader.parse_plan(upf_pb, plan)
    upf_plan_stn = upf_plan.convert_to(PlanKind.STN_PLAN, problem)

    writer = PDDLWriter(upf_pb)
    writer.write_plan(upf_plan_stn, output)

def p_plan(plan_path, robot_state, mission):
    """ 
    Parses the plan to extract:
    - The last **position** and **conf** state of each robot.
    - The **max execution time** of all robots in the plan.
    - The **individual execution times** of each robot.

    Args:
        plan_path (str): The path to the plan file.
        robot_state (dict): The dictionary tracking robot positions & states.

    Returns:
        tuple: Updated robot_state, max_execution_time, robot_execution_times
    """
    
    with open(plan_path, 'r') as file:
        plan = file.read()

    actions = plan.strip().split('\n')
    robot_execution_times = defaultdict(float)  # Stores max execution time per robot

    for action in actions:
        if action:
            time, details = action.split(":")
            start_time = float(time.strip())  # Convert time to float
            details = details.split("[")[0].strip()
            action_name, *params = details.split()
            duration = float(action.split("[")[-1].split("]")[0].strip())  # Extract action duration
            end_time = start_time + duration  # Compute action end time
            
            # Store the latest execution time per robot
            for r in robot_state.keys():
                if r in action:
                    robot_execution_times[r] = max(robot_execution_times[r], end_time)

            # **Extract final position and configuration**
            if "observe" in action_name:
                robot1 = params[0]
                if "observe_2r" in action_name:
                    robot2 = params[1]
                    robot_state[robot2]['position'] = params[-2]
                    robot_state[robot2]['conf'] = "airconf"
                    robot_state[robot2]['time'] =end_time
                    robot_state[robot2]['site'] = params[-1].split(')')[0]
                    poi_obj = next((p for s in mission.sites for p in s.poi if p.name == params[-2]), None)
                    if poi_obj:
                        robot_state[robot2]['loc'] = poi_obj.loc
                robot_state[robot1]['position'] = params[-2]
                robot_state[robot1]['conf'] = "airconf"
                robot_state[robot1]['time'] = end_time
                robot_state[robot1]['site'] = params[-1].split(')')[0]
                poi_obj = next((p for s in mission.sites for p in s.poi if p.name == params[-2]), None)
                if poi_obj:
                        robot_state[robot1]['loc'] = poi_obj.loc
            elif "navigation" in action_name or "switch" in action_name or "translate_data" in action_name:
                robot = params[0]
                robot_state[robot]['position'] = params[-2]
                robot_state[robot]['time'] = end_time  # Update last action time
                robot_state[robot]['site'] = params[-1].split(')')[0]
                poi_obj = next((p for s in mission.sites for p in s.poi if p.name == params[-2]), None)
                if poi_obj:
                        robot_state[robot]['loc'] = poi_obj.loc

                # Update robot's configuration based on action type
                if "airwater" in action_name:
                    robot_state[robot]['conf'] = 'waterconf'
                elif "waterair" in action_name:
                    robot_state[robot]['conf'] = 'airconf'
                elif "takeoff" in action_name:
                    robot_state[robot]['conf'] = 'airconf'
                elif "landing" in action_name:
                    robot_state[robot]['conf'] = 'groundconf'

            elif "change_site" in action_name :
                robot = params[0]
                robot_state[robot]['position'] = params[-1].split(')')[0]
                robot_state[robot]['time'] = end_time  # Update last action time
                robot_state[robot]['site'] = params[2]
                poi_obj = next((p for s in mission.sites for p in s.poi if p.name == params[-1].split(')')[0]), None)
                if poi_obj:
                        robot_state[robot]['loc'] = poi_obj.loc

    # Find the maximum execution time among all robots
    max_execution_time = max(robot_execution_times.values(), default=0)

    return robot_state, max_execution_time, robot_execution_times


def trim_plan_file(file_path):
    # Flag to start recording lines
    start_recording = False
    output_lines = []

    with open(file_path, 'r') as file:
        for line in file:
            # Check if the line contains the start of actions
            if line.startswith('0.000:'):
                start_recording = True
            # Once the starting line is found, start appending lines
            if start_recording:
                output_lines.append(line)

    # Optionally, write the trimmed content back to a file
    with open(file_path, 'w') as file:
        file.writelines(output_lines)


def filter_robots_in_file(input_filepath, output_filepath, allowed_robots):
    with open(input_filepath, 'r') as file:
        lines = file.readlines()
    
    filtered_lines = []
    
    for line in lines:
        if any(robot in line for robot in allowed_robots):
            filtered_lines.append(line)
        elif 'robot' not in line:
            # This keeps lines that don't mention 'robot' at all.
            filtered_lines.append(line)
    
    with open(output_filepath, 'w') as file:
        file.writelines(filtered_lines)

from typing import (
    Any,
    Dict,
    Iterable,
    List,
    Optional,
    Sequence,
    Set,
    Tuple,
    Union,
    Callable,
)

from unified_planning.plot.utils import (
    FIGSIZE,
    FIGSIZE_SCALE_FACTOR,
    ARROWSIZE,
    NODE_COLOR,
    EDGE_COLOR,
    FONT_SIZE,
    FONT_COLOR,
    EDGE_FONT_SIZE,
    EDGE_FONT_COLOR,
    draw_base_graph,
)
from unified_planning.plans.stn_plan import STNPlan, STNPlanNode

def _generate_stn_edge_label(
    lower_bound: Optional[Fraction], upper_bound: Optional[Fraction]
) -> str:
    if lower_bound is None:
        lb = "-inf"
    elif lower_bound.denominator == 1:
        lb = str(lower_bound.numerator)
    else:
        lb = str(float(lower_bound))

    if upper_bound is None:
        ub = "+inf"
    elif upper_bound.denominator == 1:
        ub = str(upper_bound.numerator)
    else:
        ub = str(float(upper_bound))
    return f"[{lb}, {ub}]"


def plot_stn_plan_robotdepend(
    plan: "STNPlan",
    *,
    filename: Optional[str] = None,
    figsize: Optional[Tuple[float, float]] = None,
    top_bottom: bool = False,
    generate_node_label: Optional[Callable[["STNPlanNode"], str]] = None,
    generate_node_color: Optional[Callable[["STNPlanNode"], str]] = None,  # Add this parameter
    arrowsize: int = ARROWSIZE,
    node_size: Optional[Union[int, Sequence[int]]] = None,
    edge_color: Union[str, Sequence[str]] = EDGE_COLOR,
    font_size: int = FONT_SIZE,
    font_color: str = FONT_COLOR,
    generate_edge_label: Optional[
        Callable[[Optional[Fraction], Optional[Fraction]], str]
    ] = None,
    edge_font_size: int = EDGE_FONT_SIZE,
    edge_font_color: str = EDGE_FONT_COLOR,
    draw_networkx_kwargs: Optional[Dict[str, Any]] = None,
    draw_networkx_edge_labels_kwargs: Optional[Dict[str, Any]] = None,
):
    import matplotlib.pyplot as plt  # type: ignore[import]

    if generate_edge_label is None:
        edge_label_function: Callable[
            [Optional[Fraction], Optional[Fraction]], str
        ] = _generate_stn_edge_label
    else:
        edge_label_function = generate_edge_label
    if generate_node_label is None:
        generate_node_label = str
    if generate_node_color is None:  # Default to a single color if no callable provided
        generate_node_color = lambda node: NODE_COLOR
    if draw_networkx_edge_labels_kwargs is None:
        draw_networkx_edge_labels_kwargs = {}

    edge_labels: Dict[Tuple[STNPlanNode, STNPlanNode], str] = {}
    graph = nx.DiGraph()
    node_colors = []

    for left_node, constraint_list in plan.get_constraints().items():
        for lower_bound, upper_bound, right_node in constraint_list:
            graph.add_edge(left_node, right_node)
            edge_labels[(left_node, right_node)] = edge_label_function(
                lower_bound, upper_bound
            )

    for node in graph.nodes:
        node_colors.append(generate_node_color(node))  # Use the callable to determine node color

    fig, ax, pos = draw_base_graph(
        graph,
        figsize=figsize,
        top_bottom=top_bottom,
        generate_node_label=generate_node_label,
        arrowsize=arrowsize,
        node_size=node_size,
        node_color=node_colors,  # Apply the dynamically generated colors
        edge_color=edge_color,
        font_size=font_size,
        font_color=font_color,
        draw_networkx_kwargs=draw_networkx_kwargs,
    )
    nx.draw_networkx_edge_labels(
        graph,
        pos,
        edge_labels=edge_labels,
        font_size=edge_font_size,
        font_color=edge_font_color,
        ax=ax,
        **draw_networkx_edge_labels_kwargs,
    )
    if filename is None:
        plt.show()
    else:
        fig.savefig(filename)



# domain = Path("/home/virgile/PHD/test_ws/MMdomainextended.pddl")
# problem = Path("/home/virgile/PHD/test_ws/site4.pddl")
# output = Path("/home/virgile/PHD/test_ws/STNoutput.txt")
# plan = Path("/home/virgile/PHD/test_ws/site4PDDLSOLVE.txt")

# reader = PDDLReader()
# upf_pb = reader.parse_problem(domain, problem)
# if upf_pb.epsilon is None:
#     upf_pb.epsilon = Fraction(1,1000)
# # actions: List[Tuple[Fraction, Any, Fraction]] = []

# upf_plan = reader.parse_plan(upf_pb, plan)
# # plot_causal_graph(upf_pb)
# # plot_time_triggered_plan(upf_plan)

# upf_plan_stn = upf_plan.convert_to(PlanKind.STN_PLAN, upf_pb)

def node_to_string(node):
    return str(node.action_instance) #.split('(')[0]

def generate_node_color(node):
    if "robot0" in str(node.action_instance):
        return "blue"
    elif "robot1" in str(node.action_instance):
        return "red"
    elif "robot2" in str(node.action_instance):
        return "green"
    elif "robot3" in str(node.action_instance):
        return "yellow"
    else:
        return "grey"



def filtrate_plan(plan):
    for act in plan._stn.distances.keys():
        print(act.kind)

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

# print(upf_plan_stn)

# plot_causal_graph(upf_plan.convert_to(PlanKind.SEQUENTIAL_PLAN, upf_pb))
# plot_partial_order_plan(upf_plan.convert_to(PlanKind.PARTIAL_ORDER_PLAN, upf_pb))

# filtrate_plan(upf_plan_stn)
# plot_stn_plan_robotdepend(upf_plan_stn,figsize=[100,100],generate_node_label=node_to_string,generate_node_color=generate_node_color,arrowsize=3,node_size=200,edge_font_size=5)

# convert_toSTN(plan=plan,output=output,domain=domain,problem=problem)

# t1 = time.perf_counter() 
# robot_info = p_plan(plan_contents)
# return_problem_file('test_ws/site4.pddl', 'test_ws/output.pddl', robot_info)
# nav_problem_file('test_ws/problem_test.pddl', 'test_ws/output2.pddl', ["robot0", "robot1"], "site4", "site6")
# filter_robots_in_file('test_ws/output2.pddl', 'test_ws/output3.pddl', ["robot0", "robot1"])
# t2 = time.perf_counter() 
# print(t2-t1)


# domain = Path("/home/virgile/PHD/test_ws/MMdomainextended.pddl")

# problem1 = Path("/home/virgile/PHD/test_ws/site4.pddl")
# problem2 = Path("/home/virgile/PHD/test_ws/site6.pddl")
# plan1 = Path("/home/virgile/PHD/test_ws/site4PDDLSOLVE.txt")
# plan2 = Path("/home/virgile/PHD/test_ws/site6PDDLSOLVE.txt")
# problems = [problem1,problem2]
# output= Path("/home/virgile/PHD/test_ws/output.txt")
# merge_for_single_path(plans=[plan1,plan2], output=output, domain=domain, problems=problems)


def find_optimal_path(cluster_sites, base_site):
    """
    Compute the optimal site visit order using TSP (Traveling Salesman Problem).
    Ensures path starts and ends at the base.
    """
    # Extract site names and positions
    site_names = [base_site.name] + [site.name for site in cluster_sites] + [base_site.name]
    site_positions = np.array([base_site.center[:2]] + [site.center[:2] for site in cluster_sites] + [base_site.center[:2]])

    # Compute distance matrix
    dist_matrix = cdist(site_positions, site_positions, metric='euclidean')

    # Solve TSP using exact dynamic programming
    tsp_path = solve_tsp_dynamic_programming(dist_matrix)[0]
    optimal_order = [site_names[i] for i in tsp_path]

    return optimal_order

def generate_possible_paths(optimal_order, num_robots):
    """
    Generate a limited number of subpaths while ensuring all sites are covered.
    - First path is always the full optimal path.
    - Subpaths are recursively split, ensuring coverage.
    - The number of subpaths does not exceed the number of robots.
    """

    base = optimal_order[0]
    sites_only = optimal_order[1:-1]  # Remove start/end base

    # Start with the full optimal path
    paths = [optimal_order]

    # If we need more paths, split the main path into two
    if num_robots > 1:
        midpoint = len(sites_only) // 2
        paths.append([base] + sites_only[:midpoint] + [base])
        paths.append([base] + sites_only[midpoint:] + [base])

    # If more robots, generate further splits ensuring all sites remain covered
    while len(paths) < num_robots:
        longest_path = max(paths, key=len)  # Find longest path to split
        if len(longest_path) <= 3:
            break  # Avoid splitting too much

        midpoint = len(longest_path) // 2
        new_path1 = longest_path[:midpoint] + [base]
        new_path2 = [base] + longest_path[midpoint:]

        # Replace the longest path with two shorter paths
        paths.remove(longest_path)
        paths.append(new_path1)
        paths.append(new_path2)

    # Limit paths to the number of robots
    paths = paths[:num_robots]

    # Debugging
    # print(f"\n🔹 Generated {len(paths)} paths for {num_robots} robots:")
    # for i, path in enumerate(paths):
    #     print(f"  Path {i+1}: {path}")

    return paths

def validate_team_allocation(paths, allocation):
    """
    Ensure that all sites are covered with at least 2 robots.
    """

    site_coverage = {site: 0 for path in paths for site in path}
    
    for path, robots in zip(paths, allocation):
        for site in path:
            site_coverage[site] += robots  # Count how many robots visit each site
    
    # Ensure all sites are visited at least twice
    return all(count >= 2 for site, count in site_coverage.items() if site != 'base')

def generate_team_allocations(num_robots, paths):
    """
    Generate valid robot-team allocations across paths.
    - Ensures at least 2 robots per site.
    - Uses dynamic allocation filtering.
    """

    path_count = len(paths)
    print(f"🔹 Generating team allocations for {num_robots} robots.")

    # Generate valid team sizes dynamically
    valid_team_sizes = [i+1 for i in range(num_robots)]
    print(f"✅ Valid team sizes: {valid_team_sizes}")

    valid_distributions = []

    # Try different assignments using itertools.combinations_with_replacement
    for comb in combinations_with_replacement(valid_team_sizes, path_count):
        if sum(comb) == num_robots and validate_team_allocation(paths, comb):
            valid_distributions.append(comb)

    print(f"✅ {len(valid_distributions)} valid team allocations found.")
    return valid_distributions




def compute_cost_for_team_assignment(paths, robot_distribution, site_dict):
    """
    Compute the cost for each robot allocation scenario.
    """
    team_costs = []

    for path, team_size in zip(paths, robot_distribution):
        path_cost = sum(distance(site_dict[path[j]], site_dict[path[j + 1]]) for j in range(len(path) - 1))
        adjusted_cost = path_cost / team_size  # Larger teams complete tasks faster
        team_costs.append(adjusted_cost)

    return max(team_costs) if team_costs else float('inf')

def improved_getbestassignforclusterandR(num_robots, costsites, listusedRgroups, cluster, base, Robots):
    """
    Optimized function to assign robots to paths using flexible path assignments.
    """
    # Step 2: Generate site dictionary with base included
    site_dict = {site.name: site.center[:2] for site in cluster}
    site_dict["base"] = base.center[:2]  # Add the base site

    # Step 1: Compute the optimal TSP path for the cluster
    optimal_order = find_optimal_path(cluster, base)

    # Step 2: Generate multiple candidate paths
    all_paths = generate_possible_paths(optimal_order, num_robots)

    # Step 3: Generate valid team allocations (dynamically selected paths)
    valid_allocations = generate_team_allocations(num_robots, all_paths)

    if not valid_allocations:
        print(f"🚨 No valid team allocations found for {num_robots} robots.")
        return ScenarioAttribution(clusterinscenario=cluster, scenario=None, numrobots=num_robots, totalcost=float('inf'))

    # Step 4: Compute cost for each valid allocation
    best_allocation = None
    best_cost = float('inf')

    for allocation in valid_allocations:
        cost = compute_cost_for_team_assignment(all_paths, allocation,site_dict)
        if cost < best_cost:
            best_cost = cost
            best_allocation = allocation

    print(f"✅ Best allocation for {num_robots} robots: {best_allocation} with cost {best_cost}")

    return ScenarioAttribution(clusterinscenario=cluster, scenario=best_allocation, numrobots=num_robots, totalcost=best_cost)

# mission = Mission(sites=[Site(poi=[Poi(mediums=[0, 1], name='cpbase', loc=(-108.0, -134.5, 0), typepoi='transition')], robots=[], name='base', center=(-108.0, -134.5, 0), size=10), Site(poi=[Poi(mediums=[1], name='cpsite1', loc=(58.0, 361.5, 1), typepoi='survey'), Poi(mediums=[-1], name='pp1', loc=(71.9, 375.9, -3), typepoi='sample'), Poi(mediums=[-1], name='pp2', loc=(59.9, 363.5, -3), typepoi='sample'), Poi(mediums=[-1], name='pp3', loc=(66.8, 348.6, -3), typepoi='sample'), Poi(mediums=[-1, 1], name='sp1', loc=(64.1, 359.9, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp2', loc=(60.0, 371.1, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp3', loc=(71.9, 375.8, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp4', loc=(43.0, 373.1, 0), typepoi='transition')], robots=[], name='site1', center=(58.0, 361.5, 0), size=30), Site(poi=[Poi(mediums=[1], name='cpsite2', loc=(871.0, 799.8, 1), typepoi='survey'), Poi(mediums=[-1], name='pp4', loc=(867.2, 794.9, -1), typepoi='sample'), Poi(mediums=[-1, 1], name='sp5', loc=(872.1, 804.5, 0), typepoi='transition')], robots=[], name='site2', center=(871.0, 799.8, 0), size=10), Site(poi=[Poi(mediums=[1], name='cpsite3', loc=(179.3, -467.0, 1), typepoi='survey'), Poi(mediums=[-1], name='pp5', loc=(164.7, -464.4, -3), typepoi='sample'), Poi(mediums=[-1], name='pp6', loc=(184.7, -477.2, -3), typepoi='sample'), Poi(mediums=[-1], name='pp7', loc=(187.5, -468.7, -3), typepoi='sample'), Poi(mediums=[-1, 1], name='sp6', loc=(168.5, -457.8, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp7', loc=(171.3, -455.9, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp8', loc=(186.1, -459.9, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp9', loc=(176.2, -473.7, 0), typepoi='transition')], robots=[], name='site3', center=(179.3, -467.0, 0), size=30), Site(poi=[Poi(mediums=[1], name='cpsite4', loc=(203.4, 191.2, 0), typepoi='survey'), Poi(mediums=[-1], name='pp8', loc=(198.7, 195.9, -1), typepoi='sample'), Poi(mediums=[-1, 1], name='sp10', loc=(206.1, 191.9, 0), typepoi='transition')], robots=[], name='site4', center=(203.4, 191.2, 0), size=10), Site(poi=[Poi(mediums=[1], name='cpsite5', loc=(639.4, -878.6, 1), typepoi='survey'), Poi(mediums=[-1], name='pp9', loc=(652.7, -865.8, -4), typepoi='sample'), Poi(mediums=[-1], name='pp10', loc=(651.2, -877.0, -4), typepoi='sample'), Poi(mediums=[-1], name='pp11', loc=(652.0, -894.0, -4), typepoi='sample'), Poi(mediums=[-1], name='pp12', loc=(640.4, -859.7, -4), typepoi='sample'), Poi(mediums=[-1, 1], name='sp11', loc=(621.9, -862.6, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp12', loc=(650.5, -864.0, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp13', loc=(644.6, -889.2, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp14', loc=(658.3, -882.2, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp15', loc=(621.2, -867.5, 0), typepoi='transition')], robots=[], name='site5', center=(639.4, -878.6, 0), size=40), Site(poi=[Poi(mediums=[1], name='cpsite6', loc=(-183.3, 750.6, 1), typepoi='survey'), Poi(mediums=[-1], name='pp13', loc=(-185.8, 755.6, -1), typepoi='sample'), Poi(mediums=[-1, 1], name='sp16', loc=(-187.7, 750.8, 0), typepoi='transition')], robots=[], name='site6', center=(-183.3, 750.6, 0), size=10), Site(poi=[Poi(mediums=[1], name='cpsite7', loc=(412.1, 485.7, 1), typepoi='survey'), Poi(mediums=[-1], name='pp14', loc=(416.5, 484.3, -3), typepoi='sample'), Poi(mediums=[-1], name='pp15', loc=(403.7, 489.3, -3), typepoi='sample'), Poi(mediums=[-1], name='pp16', loc=(417.7, 494.8, -3), typepoi='sample'), Poi(mediums=[-1, 1], name='sp17', loc=(421.5, 487.1, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp18', loc=(402.7, 484.5, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp19', loc=(414.0, 478.5, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp20', loc=(417.7, 479.1, 0), typepoi='transition')], robots=[], name='site7', center=(412.1, 485.7, 0), size=30)], robots=[Robot(name='robot0', loc=(-108.0, -134.5, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot1', loc=(-108.0, -134.5, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot2', loc=(-108.0, -134.5, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot3', loc=(-108.0, -134.5, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot4', loc=(-108.0, -134.5, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot5', loc=(-108.0, -134.5, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot6', loc=(-108.0, -134.5, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000)], objective='assess', arenasize=1000, sitesize=(10, 50))

def find_sequential_links(paths, mission):
    """
    Identifies sequential dependencies between paths based on robot movement order.

    Returns:
        - A list of (path_x, path_y) tuples indicating that path_x must finish before path_y.
    """
    sequential_links = []
    path_dependencies = defaultdict(set)

    # Create a mapping of robots to their mission sequence order
    robot_order = {r.name: r.histo for r in mission.robots}

    # Map robots to paths they appear in
    robot_to_paths = defaultdict(list)
    for i, path_info in enumerate(paths):
        for robot in path_info["robots"]:
            robot_to_paths[robot].append(i)  # Store path index for each robot

    # print("Robot-to-Path Mapping:", robot_to_paths)

    # Identify sequential dependencies
    for robot, robot_paths in robot_to_paths.items():
        # Sort paths based on the robot's **actual mission order**
        ordered_paths = sorted(robot_paths, key=lambda p: robot_order[robot].index(paths[p]["path"][0]))

        # Establish sequential dependencies
        for j in range(len(ordered_paths) - 1):
            prev_path = ordered_paths[j]
            next_path = ordered_paths[j + 1]

            # Ensure dependencies go from **earlier to later in mission sequence**
            if next_path not in path_dependencies[prev_path]:  # Prevent duplicate dependencies
                path_dependencies[prev_path].add(next_path)
                sequential_links.append((prev_path, next_path))

    return sequential_links

def split_redundant_paths(unique_paths_with_robots, mission):
    new_paths = []
    for path_info in unique_paths_with_robots:
        split_path = []
        for site_name in path_info["path"]:
            site = next((s for s in mission.sites if s.name == site_name), None)

            # Check if the robots assigned to the site differ from the current path robots
            site_robots = {r.name for r in site.robots}
            path_robots = set(path_info["robots"])

            # if len(path_robots) == 1 and :
            #     new_paths.append({'path': [site_name], 'robots': list(path_robots)})
            
            if site_robots != path_robots:
                # If the site robots are different, split it into a new path
                if {'path': [site_name], 'robots': list(site_robots)} not in new_paths:
                    new_paths.append({'path': [site_name], 'robots': list(site_robots)})

            else:
                split_path.append(site_name)

        # Add the original path if not empty after filtering
        if split_path and {'path': split_path, 'robots': path_info["robots"]} not in new_paths:
            new_paths.append({'path': split_path, 'robots': path_info["robots"]})

    return new_paths

def extract_execution_time(plan_file):
    """ Extracts total execution time from the generated PDDL plan. """
    execution_time = 0
    with open(plan_file, 'r') as file:
        for line in file:
            if "; Time" in line:
                execution_time = float(line.split(":")[-1].strip())  # Extract numerical time value
    return execution_time

def update_STN_temporal_links(STN, executed_path, execution_time):
    """ Updates the STN by setting execution time constraints to dependent paths. """

    if executed_path in STN.nodes:
        # Update the executed node's latest_finish time
        STN.nodes[executed_path]["latest_finish"] = execution_time

        for successor in list(STN.successors(executed_path)):
            # Ensure successor's earliest start is at least the execution_time
            STN.nodes[successor]["earliest_start"] = max(
                STN.nodes[successor]["earliest_start"],
                execution_time
            )

            # **Fix missing edge times for sync paths**
            if (executed_path, successor) in STN.edges:
                time_gap = execution_time - STN.nodes[executed_path]["earliest_start"]
                STN.edges[executed_path, successor]["min_time_gap"] = max(0, time_gap)

            # Ensure successor's latest finish is correctly propagated
            STN.nodes[successor]["latest_finish"] = max(
                STN.nodes[successor]["latest_finish"],
                execution_time
            )

def get_executable_paths(STN):
    executable_paths = []
    for node in STN.nodes:
        
        if node in ["Start", "End"]:
            continue  # Skip start/end nodes
        

        # Check if all predecessors are executed
        predecessors = list(STN.predecessors(node))
        if all(STN.nodes[p]["executed"]== True for p in predecessors) and (STN.nodes[node]["executed"] == False):
            print
            executable_paths.append(node)

    return executable_paths
# bfs_layout

def draw_STN(STN):
    """
    Draws the STN graph properly formatted:
    - Nodes are labeled with sites, not path indices.
    - "Start" is on the left, "End" on the right.
    - Execution constraints (`earliest_start`, `latest_finish`) are shown on nodes.
    - Edges show `min_time_gap` correctly.
    """

    # **Force Graphviz Layout Using the graphviz Module**
    dot = graphviz.Digraph(format='png')  # Create a Graphviz Digraph

    # **Fix Node Labels: Show site names instead of Path X**
    labels = {}
    for node in STN.nodes:
        if node == "Start":
            labels[node] = "Start"
        elif node == "End":
            labels[node] = "End"
        else:
            site_names = ", ".join(STN.nodes[node].get("sites", []))  # Get site names
            robot_names = ", ".join(STN.nodes[node].get("robots", []))
            earliest = STN.nodes[node].get("earliest_start", 0)
            latest = STN.nodes[node].get("latest_finish", 0)
            labels[node] = f"Path {node}\n[{site_names}]\n[{robot_names}]"  # Proper formatting

        # Add nodes to Graphviz with labels
        dot.node(str(node), labels[node], shape="ellipse", style="filled", fillcolor="lightblue")

    # **Fix Edge Labels: Show min_time_gap**
    for u, v in STN.edges:
        time_gap = STN.edges[u, v].get("min_time_gap", "0")
        time_gap = round(time_gap,2)
        dot.edge(str(u), str(v), label=f"{time_gap}s", color="red")

    # **Render and Display the Graph**
    dot.render('/tmp/stn_graph')  # Save to a temporary location
    plt.figure(figsize=(20, 10))
    img = plt.imread('/tmp/stn_graph.png')  # Load the rendered image
    plt.imshow(img)
    plt.axis('off')  # Hide axis for clean visualization
    plt.title("Structured Simple Temporal Network (STN)")
    plt.show()


def build_STN(paths, sequential_links):
    """
    Constructs a Simple Temporal Network (STN) from paths and sequential dependencies.
    - Adds a "Start" node linking to all paths with no predecessors.
    - Adds an "End" node linked from all paths with no successors.
    """

    STN = nx.DiGraph()  # Directed graph for STN

    # Add paths as nodes
    for i, path_info in enumerate(paths):
        STN.add_node(i, 
                     path_id=i, 
                     sites=path_info["path"], 
                     robots=path_info["robots"],
                     earliest_start=0, 
                     latest_finish=float('inf'),
                     executed=False)

    # Add edges (dependencies between paths)
    for prev_path, next_path in sequential_links:
        STN.add_edge(prev_path, next_path, min_time_gap=0)

    # Identify Start and End Paths
    initial_paths = {i for i in range(len(paths))} - {link[1] for link in sequential_links}
    final_paths = {i for i in range(len(paths))} - {link[0] for link in sequential_links}

    # Add Start Node
    STN.add_node("Start", earliest_start=0, latest_finish=0, executed=True)
    for init_path in initial_paths:
        STN.add_edge("Start", init_path, min_time_gap=0)

    # Add End Node
    STN.add_node("End", earliest_start=float('inf'), latest_finish=float('inf'))
    for final_path in final_paths:
        STN.add_edge(final_path, "End", min_time_gap=0)

    return STN
def extract_paths_with_dependencies(stn):
    paths = {}  # Dictionary to store paths with assigned robots
    dependencies = {}  # Dictionary to store path dependencies
    visited_nodes = set()

    # Identify valid start nodes (excluding "Start" and "End")
    start_nodes = [n for n in stn.nodes() if "Start" in stn.predecessors(n)]

    for start_node in start_nodes:
        queue = [start_node]

        while queue:
            node = queue.pop(0)  # Process nodes in BFS/DFS order

            if node in visited_nodes or node in ["Start", "End"]:
                continue  # Skip already visited and special nodes
            visited_nodes.add(node)

            # Retrieve node information
            node_data = stn.nodes[node]
            sites = node_data["sites"]
            robots = node_data["robots"]

            # Store path information
            paths[node] = {
                "sites": sites,  # Sites assigned to the path
                "robots": robots  # Robots assigned to the path
            }

            # Get predecessors (excluding "Start")
            pred = [p for p in stn.predecessors(node) if p != "Start"]
            dependencies[node] = pred if pred else []  # Store predecessors

            # Add successors to queue (excluding "Start" and "End")
            successors = [s for s in stn.successors(node) if s not in ["Start", "End"]]
            queue.extend(successors)

    # **Sorting Fix**: Separate numeric and sync paths, then sort them correctly
    def custom_sort_key(item):
        key, _ = item
        if isinstance(key, int):  # Normal integer path indices
            return (0, key)
        if key.startswith("sync_"):  # Sync paths (keep their order)
            return (1, key)
        return (2, key)  # Fallback for unexpected cases

    # Print paths in required format
    print("\nUnique paths in plan (with dependencies):")
    for idx, data in sorted(paths.items(), key=custom_sort_key):
        print(f"path {idx}: {data['sites']} robots: {data['robots']} | Predecessors: {dependencies[idx]}")

