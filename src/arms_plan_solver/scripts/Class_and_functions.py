import itertools
import os
import time
from dataclasses import dataclass, field
from math import sqrt
from random import randint, randrange, uniform
import scipy.stats as scistat
import subprocess
import json

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.patches as mpatches
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
        list_of_sites_center.append(s.poi[0])
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
        
        # costcluster = CostEstimOftour(cluster)[1]
        for s in cluster:
            for pt in s.poi:
                # costcluster+=  CostEstimOftour()[1] 
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
        plt.plot(Xr,Yr,marker = '.', color=c, label='cluster {0} order'.format(idx))
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

def get_weights_of_sites(sites=[],robots=[],listrused=[]):
    """ Use sites and possible teams as input and send back the associated weights and cost estimation based on local clustering of objectives and TSP solving"""
    
    if listrused == []:
        listrused = getlistusedRgroups(robots)

    weights_sites_list=[]
    costs_of_all_sites=[]
    for s in sites:
        cost_of_site = []
        maxclust= min(len(robots)+1,len(s.poi))
        
        for r in listrused:
            costtours=[]
            clusters = Clustersite(poi=s.poi, nb_cluster=maxclust, weightslist=None)
            for cluster in clusters:
                costtours.append(CostEstimOftour(cluster)[1])
                
            cost_of_site.append(max(costtours))

        # If the sites doesnt contain more that 2 objectives, we put a very small cost on it 
        if maxclust <2:
            weights_sites_list.append(1.0)
            continue

        costs_of_all_sites.append(cost_of_site)
        weights_sites_list.append(sum(cost_of_site))

    return weights_sites_list, costs_of_all_sites

def CostEstimOftour(poi=[],waterspeed =0.5, airspeed = 1.5, assesscost=30, explorecost=15, commcost=45, switchcost=10):    
    """ Calculate an estimate cost of a robot 'Tour' for a certain number of POI with taking into account POI specifics """

    distance_pt_matrix_site = np.zeros((len(poi), (len(poi))))
    for onepoi in range(len(poi)):
        for twopoi in range(len(poi)):
            
            if poi[onepoi].name != poi[twopoi].name:
                
                if poi[onepoi].typepoi == "transition" and poi[twopoi].typepoi == "transition":
                    distance_pt_matrix_site[onepoi][twopoi]=min((distance(poi[onepoi].loc,poi[twopoi].loc) / waterspeed) + explorecost,(distance(poi[onepoi].loc,poi[twopoi].loc) / airspeed) + 2*switchcost+ explorecost)
                
                elif poi[onepoi].typepoi == "transition" and poi[twopoi].typepoi =="sample":
                    distance_pt_matrix_site[onepoi][twopoi]=(distance(poi[onepoi].loc,poi[twopoi].loc) / waterspeed )+ assesscost

                elif poi[onepoi].typepoi == "transition" and poi[twopoi].typepoi =="survey":
                    distance_pt_matrix_site[onepoi][twopoi]=(distance(poi[onepoi].loc,poi[twopoi].loc) / airspeed)

                elif poi[onepoi].typepoi == "survey" and poi[twopoi].typepoi =="transition":
                    distance_pt_matrix_site[onepoi][twopoi]=(distance(poi[onepoi].loc,poi[twopoi].loc) / airspeed )+ explorecost

                elif poi[onepoi].typepoi == "survey" and poi[twopoi].typepoi =="survey":
                    distance_pt_matrix_site[onepoi][twopoi]=(distance(poi[onepoi].loc,poi[twopoi].loc) / airspeed )

                elif poi[onepoi].typepoi == "sample" and poi[twopoi].typepoi =="transition":
                    distance_pt_matrix_site[onepoi][twopoi]=(distance(poi[onepoi].loc,poi[twopoi].loc) / waterspeed)

                elif poi[onepoi].typepoi == "sample" and poi[twopoi].typepoi =="sample":
                    distance_pt_matrix_site[onepoi][twopoi]=(distance(poi[onepoi].loc,poi[twopoi].loc) / waterspeed) + assesscost

    # distance_pt_matrix_site[:, 0] = 0 #open path problem, change ?
    return solve_tsp_dynamic_programming(distance_pt_matrix_site)

def Clustersite(poi=[], nb_cluster=int, weightslist=None):
    """ Cluster the POI into 'nb_cluster' number of clusters"""

    # '''
    # Clusters poi into X (number of clusters) clusters.

    # Parameters:
    #     Site: (list of poi)

    # Returns:
    #     clusters (X list): clusters of points indexed for each robot:
    # '''

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

import os

def GenerateAdaptedProblem(previous_problem, robot_state, current_site, next_site):
    """ 
    Generate a sub-problem PDDL file for each robot following the mission scenario.
    
    This function modifies an existing PDDL problem file by:
    - Keeping only the robots in `robot_state`
    - Updating `at`, `at_site`, and `conf` predicates based on the `current_site`
    - Modifying the `goal` to ensure the robots sample `next_site`
    - Writing the new problem to `/tmp/subproblems/`
    """
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
            new_lines.append("(define (problem subp_{})\n".format(current_site.name + next_site.name))
            continue 

         # Handle point of interest definition
        elif "- pointofinterest" in line:
            tmpline = line.split(" ")
            filtered_pois = [exp for exp in tmpline if any(exp == poi.name for poi in current_site.poi) or any(exp == poi.name for poi in next_site.poi)]
            new_lines.append(" ".join(filtered_pois) + " - pointofinterest\n")
            continue
        
        # Handle site definition
        elif "- site" in line:
            tmpline = line.split(" ")
            filtered_sites = [exp for exp in tmpline if exp == current_site.name or exp == next_site.name]
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
                    if all(not v for v in robot_state.values()):
                        if 'at {} '.format(robot) in line:
                            new_lines.append("( at {0} cp{1} )\n".format(robot, current_site.name))
                            modified = True
                        elif 'at_site {} '.format(robot) in line:
                            new_lines.append("( at_site {0} {1} )\n".format(robot, current_site.name))
                            modified = True
                        elif 'conf {} '.format(robot) in line:
                            if current_site.name == "base":
                                new_lines.append("( groundconf {0} )\n".format(robot))
                            else:
                                new_lines.append("( airconf {0} )\n".format(robot))
                            modified = True
                    else:
                        if 'at {} '.format(robot) in line:
                            new_lines.append("( at {0} {1} )\n".format(robot, robot_state[robot]["position"]))
                            modified = True
                        elif 'at_site {} '.format(robot) in line:
                            new_lines.append("( at_site {0} {1} )\n".format(robot, current_site.name))
                            modified = True
                        elif 'conf {} '.format(robot) in line:
                            new_lines.append("( {0} {1} )\n".format(robot_state[robot]["conf"], robot))
                            modified = True
                
                if not modified:
                    new_lines.append(line + "\n")  # If not modified, keep the original line
            else:
                continue
        # Filter out POI distance definitions that don't involve the current or next site
        elif "sp" in line or "pp" in line or "cp" in line:
            tmpline = line.split(" ")
            poi_in_line = [exp for exp in tmpline if "sp" in exp or "pp" in exp or "cp" in exp]
            
            # Check if all POIs in this line belong to either current or next site
            if all(any(exp == poi.name for poi in current_site.poi) or any(exp == poi.name for poi in next_site.poi) for exp in poi_in_line):
                new_lines.append(line + "\n")
            continue  # Skip lines that reference POIs from other sites
        
        elif "site_size" in line:
            if current_site.name in line or next_site.name in line:
                new_lines.append(line + "\n")
            continue  # Remove other site size definitions

        # Modify the goal section
        elif 'goal' in line and not goal_added:
            new_lines.append("(:goal\n")  # Define goal block
            new_lines.append("( and\n")            
            if next_site.name == "base":
                for robot in robot_state.keys():
                    new_lines.append("( groundconf {0})\n".format(robot))
            else:
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
    updated_path = os.path.join(output_dir, f"subp_{current_site.name}_{next_site.name}")

    # Write the updated problem file
    with open(updated_path + ".pddl", 'w') as file:
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

def Weightned_Cluster(mission=Mission(robots=[],sites=[]),numb_clt=2):
    """ Create and balance the clusters with KMEAN into cluster of similar weight """
    
    # We get a list of sites center for Kmean clustering
    list_of_sites_center= getcenterpoiofsites(mission.sites[1::])
    # We use Kmean clustering on sites center to get a first allocation
    clustering_Kmean = Clustersite(list_of_sites_center,numb_clt)
    # print("\n", "clustering :",clustering_Kmean,"\n")

    # clustering was done with centers, we allocate the corresponding sites in lists
    clusters_list=[]
    for cl in range(len(clustering_Kmean)):
        cluster_sites=Cluster_class(name="cluster{}".format(cl))
        for i in clustering_Kmean[cl]:
            for s in mission.sites[1:]:
                if s.poi[0] == i:
                    cluster_sites.sites.append(s)
        
        clusters_list.append(cluster_sites)

    # print("\n", "clusters_list :", clusters_list,"\n")

    # We calculate the clusters estimated cost of cluster for reallocation
    cltcosts=[] # contain the estimated cost of cluster
    cltweights=[] # contain the estimated cost of sites in cluster
    for cl in range(len(clusters_list)):

        # We get estimation of sites weights and cost for every possible number of robot in team
        weights_of_sites, _ = get_weights_of_sites(sites= clusters_list[cl].sites, robots=mission.robots)

        # Cost of TSP to visit sites + sites cost
        clustercost = CostEstimOftour(poi=clustering_Kmean[cl])[1] + sum(weights_of_sites)
        cltweights.append(clustercost)
        cltcosts.append(weights_of_sites)

    if numb_clt==1 or len(mission.sites) <=3:
        return clusters_list, cltweights, cltcosts
    
    # print("in new function show cluster and weights=",len(clusters_list[0]),len(clusters_list[1]),cltweights)
    # showclusters(mission, clusters_list, base=mission.sites[0]) 
    variation=abs(max(cltweights)-min(cltweights))
    Kmeanvariation=variation
    Kmeanclusters=clusters_list[:]
    Kmeancltweight=cltweights
    Kmeancltcost=cltcosts
    
    # allocation for reattribution
    cluster_realloc=clusters_list
    while True:      

        
        # We reallocate the sites with minimal cost to higher cost cluster until swapping no longer reduce the margin
        maxweightidx=cltweights.index(max(cltweights))
        mini= cltcosts[maxweightidx].index(min(cltcosts[maxweightidx]))
        cluster_realloc[cltweights.index(min(cltweights))].sites.append(cluster_realloc[maxweightidx].sites[mini])
        cluster_realloc[cltweights.index(max(cltweights))].sites.remove(cluster_realloc[maxweightidx].sites[mini])

        # We recalculate the cost
        cltcosts=[] # contain the estimated cost of cluster
        cltweights=[] # contain the estimated cost of sites in cluster
        for cl in range(len(cluster_realloc)):
            
            # We get estimation of sites weights and cost for every possible number of robot in team
            weights_of_sites, _ = get_weights_of_sites(sites= cluster_realloc[cl].sites, robots=mission.robots)
            # Cost of TSP to visit sites + sites cost
            new_list_of_sites_center = getcenterpoiofsites(cluster_realloc[cl].sites)
            clustercost = CostEstimOftour(poi=new_list_of_sites_center)[1] + sum(weights_of_sites)
            cltweights.append(clustercost)
            cltcosts.append(weights_of_sites)
            
        swap_variation=abs(max(cltweights)-min(cltweights))
        if swap_variation > variation:
            break
        else:
            variation=swap_variation
            clusters_list=cluster_realloc

    if variation <= Kmeanvariation:
        # print("weighted choice")
        return clusters_list, cltweights, cltcosts
    else:
        # print("kmean choice")
        return Kmeanclusters, Kmeancltweight,Kmeancltcost


def calculate_path_cost_estim_sites(sce= ScenarioAttribution(clusterinscenario=[],scenario=(2,2)), Robots=[], Sites = [], clst=False, listusedRgroups=[], assign=False, base=Site(poi=[], robots=[])):
    """ Calculate the estimate cost of a scenario of allocation when considering sites TODO: mix with cluster one """


    # print([i.name for i in Sites])
    idx=0
    for assignment, site in zip(sce.scenario, Sites):
        # print(site.name, )
        costaction= sce.costmatrix[idx][listusedRgroups.index(assignment)]
        # print(costaction, assignment, sce.scenario)

        available_robots = Robots[:]

        while len(site.robots) < assignment:
            assigncost=[]
            for r in available_robots:
                
                assigncost.append(costaction + r.currentpathcost + distance(r.loc, site.center)/1.5)

            site.robots.append(available_robots[assigncost.index(min(assigncost))])
            available_robots[assigncost.index(min(assigncost))].currentpathcost +=min(assigncost)
            available_robots[assigncost.index(min(assigncost))].loc = site.center
            available_robots[assigncost.index(min(assigncost))].histo.append(site.name)
            available_robots[assigncost.index(min(assigncost))].site = site.name
            available_robots.remove(available_robots[assigncost.index(min(assigncost))])
        idx+=1
    
    test=0
    if assign == False:
        for r in Robots:
            if r.currentpathcost > test:
                sce.totalcost= r.currentpathcost
                test=r.currentpathcost
        for r in Robots:
            r.currentpathcost =0
            r.histo = []
            r.loc = base.center
        for s in Sites:
            s.robots=[]
        
    # print(sce.totalcost)
    return sce

def calculate_path_cost_estim_cluster(sce= ScenarioAttribution(clusterinscenario=[],scenario=(2,2)), Robots=[], Cluster = [], listusedRgroups=[], base=Site(poi=[], robots=[])):
    """ Calculate the estimate cost of a scenario of allocation when considering clusters TODO: mix with cluster one """

    idx=0
    for assignment, cluster in zip(sce.scenario, Cluster):
        # print(site.name, )
        
        costaction= sce.costmatrix[idx][listusedRgroups.index(assignment)]
        # print(costaction, assignment,sce.scenario)
        available_robots = Robots[:]

        while len(cluster.robots) < assignment:
            assigncost=[]
            for r in available_robots:
                
                assigncost.append(costaction + r.currentpathcost + distance(r.loc, cluster.sites[0].center)/1.5)

            cluster.robots.append(available_robots[assigncost.index(min(assigncost))])
            available_robots[assigncost.index(min(assigncost))].currentpathcost+=min(assigncost)
            available_robots[assigncost.index(min(assigncost))].loc = cluster.sites[-1].center
            available_robots[assigncost.index(min(assigncost))].histo.append(cluster.name)
            available_robots.remove(available_robots[assigncost.index(min(assigncost))])
        idx+=1
    
    test=0
    for r in Robots:
        if r.currentpathcost > test:
            sce.totalcost= r.currentpathcost
            test=r.currentpathcost

    for r in Robots:
        r.currentpathcost =0
        r.histo = []
        r.loc = base.center

    for c in Cluster:
        c.robots=[]
        for s in c.sites:
            s.robots=[]

    # print("\n")
    # print("Responsable erreur de merde", len(Robots))
    # print("\n")

    return sce

def getbestassignforclusterandR(num_robots=0, costsites=np.array([]),listusedRgroups=[], cluster=[], base=Site(poi=[], robots=[]),clst=False,Robots=[]):
    """ Go through all possible assignement scenario for a given number of sites and robots and select the optimal allocation """    
    
    # Example data
    cost_matrix = costsites
    num_sites = len(cost_matrix)
    # Generate all possible robot assignments to sites
    possible_assignments = list(itertools.product(listusedRgroups, repeat=num_sites))
    # print("in get best assign cluster and R",num_robots, possible_assignments)
    # Calculate and store the cost for each generated scenario
    scenario_costs_dict = {}
    scenariosdict= {}
    for assignment in possible_assignments:
        sce = ScenarioAttribution(clusterinscenario=cluster,scenario=assignment,totalcost=0.0,costmatrix=cost_matrix)
        # scenariosdict[assignment] = calculate_path_cost_estim(sce, base=base, clst=clst, listusedRgroups=listusedRgroups)
        # print(calculate_path_cost_estim_sites(sce,Robots=Robots,Sites=cluster,listusedRgroups=listusedRgroups).totalcost)
        if clst == False:
            scenariosdict[assignment] = calculate_path_cost_estim_sites(sce,Robots=Robots[:num_robots],Sites=cluster,listusedRgroups=listusedRgroups,base=base)
        else:
            scenariosdict[assignment] = calculate_path_cost_estim_cluster(sce,Robots=Robots[:num_robots],Cluster=cluster,listusedRgroups=listusedRgroups,base=base)
        scenario_costs_dict[assignment] = sce.totalcost
        # print("scenario and cost",scenariosdict[assignment].scenario, scenario_costs_dict[assignment])
        
    # Find the optimal scenario (minimum cost)
    # if clst == True:
    #     print(scenario_costs_dict,"\n")
    
    min_cost = min(scenario_costs_dict.values())
    optimal_cost = max((k for k, v in scenario_costs_dict.items() if v == min_cost), key=sum)
    optimal_scenario = scenariosdict[optimal_cost]
    return optimal_scenario
           
def Assign_robots_to_scenario(scenario, Robots=[], Clusters = [], costsites=[],listusedRgroups=[], base=Site(poi=[], robots=[])):
    """ Assign robots to a given scenario of cluster and sites allocation """
    
    # print("\n IN ASSIGN \n")
    # print(scenario.costmatrix, scenario.scenario)
    idx=0
    for assignment, cluster in zip(scenario.scenario, Clusters):
        cluster.centroid_of_sites_2d
        available_robots = Robots[:]
        
        costaction=scenario.costmatrix[idx][listusedRgroups.index(assignment)]
        # print(scenario.costmatrix[idx][listusedRgroups.index(assignment)])
        while len(cluster.robots) < assignment:
            
            assigncost=[]
            for r in available_robots:
                # assigncost.append(r.currentpathcost + distance(r.loc,site.center)/1.5)
                assigncost.append(costaction+r.currentpathcost + distance(r.loc, cluster.sites[0].center)/1.5)

            cluster.robots.append(available_robots[assigncost.index(min(assigncost))])
            available_robots[assigncost.index(min(assigncost))].currentpathcost+=min(assigncost)
            available_robots[assigncost.index(min(assigncost))].loc = cluster.sites[0].center
            available_robots.remove(available_robots[assigncost.index(min(assigncost))])
            # available_robots[assigncost.index(min(assigncost))].histo.append(cluster.name)
            # available_robots[assigncost.index(min(assigncost))].site = site.name
        
        for sassign, site in zip(cluster.bestscenario[str(assignment)], cluster.sites):
            available_robots_in_site = cluster.robots[:]
            while len(site.robots) < sassign:
                siteassicost=[]
                for r1 in available_robots_in_site:
                    cost = r1.currentpathcost + ( distance(r1.loc, site.center)/1.5)
                    siteassicost.append(cost)

                site.robots.append(available_robots_in_site[siteassicost.index(min(siteassicost))])
                available_robots_in_site[siteassicost.index(min(siteassicost))].currentpathcost +=min(siteassicost)
                available_robots_in_site[siteassicost.index(min(siteassicost))].loc = site.center
                available_robots_in_site[siteassicost.index(min(siteassicost))].histo.append(site.name)
                available_robots_in_site[siteassicost.index(min(siteassicost))].site = site.name
                available_robots_in_site.remove(available_robots_in_site[siteassicost.index(min(siteassicost))])
        idx+=1
    # print("\n OUT ASSIGN \n")
    return scenario

def getbestassignfrommission(mission=Mission(sites=[], robots=[]), minnbofcluster=2, get_all_time=True):
    """ Main algorithm, Clusterize a number of sites into equal weight clusters and identify the best scenario of pre-allocation possible for a number of robots and number of sites  """ 

    t_alloc_full_0 = time.perf_counter()
    
    t_clustering_0 = time.perf_counter()  
    listusedRgroups = getlistusedRgroups(robots=mission.robots)
    # print(minnbofcluster,len(mission.sites)//2 +1)
    if minnbofcluster ==1 or len(mission.sites) <=2:
        clt, cltwei, cltcos=Weightned_Cluster(mission=mission,numb_clt=1)
        clusterslist=[clt]
    
    elif len(mission.sites) <=3:
        clusterslist=[]
        for numcl in range(minnbofcluster,len(mission.sites)):
            clt, cltwei, cltcos=Weightned_Cluster(mission=mission,numb_clt=numcl)
            clusterslist.append(clt)

    else:
        clusterslist=[]
        # for numcl in range(minnbofcluster,len(mission.sites)//2 +1):
        # for numcl in range(1,len(mission.robots)+1):
        for numcl in range(minnbofcluster,len(mission.sites)//2 +1):
            clt, cltwei, cltcos=Weightned_Cluster(mission=mission,numb_clt=numcl)
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
            _, clustersitescost=get_weights_of_sites(sites=cluster.sites, robots=mission.robots,listrused=listusedRgroups)
            # print("New cluster of size:", len(cluster.sites))
            # print("All cost for spectific number of robots",clustersitescost)
            scenarioclusterlist=[]
            scenarioclusterlist_costmatrix=[]

            for r in listusedRgroups:
                sce = getbestassignforclusterandR(num_robots=r, costsites=clustersitescost, listusedRgroups=listusedRgroups[:(listusedRgroups.index(r)+1)], cluster=cluster.sites, base=mission.sites[0], Robots=mission.robots)
                # print(sce.totalcost, sce.scenario)
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
        sce = getbestassignforclusterandR(num_robots=listusedRgroups[-1], Robots=mission.robots, costsites=XCclustercost, listusedRgroups=listusedRgroups, cluster=CCluster, base=mission.sites[0], clst=True)
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

def p_plan(plan_path,robot_state):
    """ Parses the plan to find the last position and state of each robot. """

    with open(plan_path, 'r') as file:
        plan = file.read()

    actions = plan.strip().split('\n')
    for action in actions:
        if action:
            time, details = action.split(":")
            details = details.split("[")[0].strip()
            action_name, *params = details.split()
            if "observe" in action_name:
                robot1 = params[0]
                if "observe_2r" in action_name:
                    robot2 = params[1]
                    robot_state[robot2] = {'position': params[-2], 'conf': "airconf"}
                robot_state[robot1] = {'position': params[-2], 'conf': "airconf"}
            elif "navigation" in action_name or "switch" in action_name:
                robot = params[0]
                if robot not in robot_state:
                    robot_state[robot] = {'position': None, 'conf': None}
                robot_state[robot]['position'] = params[-2]  # assuming last position before ']'
                if "airwater" in action_name:
                    robot_state[robot]['conf'] = 'waterconf'
                elif "waterair" in action_name:
                    robot_state[robot]['conf'] = 'airconf'
                elif "takeoff" in action_name:
                    robot_state[robot]['conf'] = 'airconf'
                elif "landing" in action_name:
                    robot_state[robot]['conf'] = 'groundconf'

    return robot_state

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



# mission = Mission(sites=[Site(poi=[Poi(mediums=[0, 1], name='cpbase', loc=(-469.2, -197.9, 0), typepoi='transition')], robots=[], name='base', center=(-469.2, -197.9, 0), size=10), Site(poi=[Poi(mediums=[1], name='cpsite1', loc=(-398.3, -73.0, 1), typepoi='survey'), Poi(mediums=[-1], name='pp1', loc=(-415.3, -58.1, -4), typepoi='sample'), Poi(mediums=[-1], name='pp2', loc=(-406.8, -77.4, -4), typepoi='sample'), Poi(mediums=[-1], name='pp3', loc=(-401.4, -60.4, -4), typepoi='sample'), Poi(mediums=[-1], name='pp4', loc=(-415.6, -62.2, -4), typepoi='sample'), Poi(mediums=[-1, 1], name='sp1', loc=(-416.2, -90.3, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp2', loc=(-413.2, -87.0, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp3', loc=(-381.0, -71.6, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp4', loc=(-415.6, -83.8, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp5', loc=(-383.7, -84.8, 0), typepoi='transition')], robots=[], name='site1', center=(-398.3, -73.0, 0), size=40), Site(poi=[Poi(mediums=[1], name='cpsite2', loc=(-225.7, 450.5, 1), typepoi='survey'), Poi(mediums=[-1], name='pp5', loc=(-220.8, 464.5, -3), typepoi='sample'), Poi(mediums=[-1], name='pp6', loc=(-213.7, 463.9, -3), typepoi='sample'), Poi(mediums=[-1], name='pp7', loc=(-228.0, 442.0, -3), typepoi='sample'), Poi(mediums=[-1, 1], name='sp6', loc=(-231.1, 443.0, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp7', loc=(-236.0, 440.5, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp8', loc=(-229.0, 436.5, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp9', loc=(-215.4, 442.9, 0), typepoi='transition')], robots=[], name='site2', center=(-225.7, 450.5, 0), size=30), Site(poi=[Poi(mediums=[1], name='cpsite3', loc=(-78.4, 412.1, 1), typepoi='survey'), Poi(mediums=[-1], name='pp8', loc=(-61.9, 403.5, -4), typepoi='sample'), Poi(mediums=[-1], name='pp9', loc=(-84.1, 413.1, -4), typepoi='sample'), Poi(mediums=[-1], name='pp10', loc=(-72.7, 422.5, -4), typepoi='sample'), Poi(mediums=[-1], name='pp11', loc=(-83.6, 395.0, -4), typepoi='sample'), Poi(mediums=[-1, 1], name='sp10', loc=(-66.9, 420.4, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp11', loc=(-81.6, 420.2, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp12', loc=(-62.5, 413.9, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp13', loc=(-94.2, 395.2, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp14', loc=(-73.4, 402.1, 0), typepoi='transition')], robots=[], name='site3', center=(-78.4, 412.1, 0), size=40), Site(poi=[Poi(mediums=[1], name='cpsite4', loc=(-311.5, 277.3, 1), typepoi='survey'), Poi(mediums=[-1], name='pp12', loc=(-317.3, 283.7, -3), typepoi='sample'), Poi(mediums=[-1], name='pp13', loc=(-314.6, 292.0, -3), typepoi='sample'), Poi(mediums=[-1], name='pp14', loc=(-315.6, 289.3, -3), typepoi='sample'), Poi(mediums=[-1, 1], name='sp15', loc=(-319.5, 285.5, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp16', loc=(-314.2, 266.4, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp17', loc=(-318.4, 281.1, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp18', loc=(-320.0, 266.3, 0), typepoi='transition')], robots=[], name='site4', center=(-311.5, 277.3, 0), size=30), Site(poi=[Poi(mediums=[1], name='cpsite5', loc=(-310.5, -120.8, 1), typepoi='survey'), Poi(mediums=[-1], name='pp15', loc=(-318.0, -127.4, -2), typepoi='sample'), Poi(mediums=[-1], name='pp16', loc=(-305.4, -128.9, -2), typepoi='sample'), Poi(mediums=[-1, 1], name='sp19', loc=(-316.6, -121.5, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp20', loc=(-308.8, -127.6, 0), typepoi='transition')], robots=[], name='site5', center=(-310.5, -120.8, 0), size=20), Site(poi=[Poi(mediums=[1], name='cpsite6', loc=(30.8, 138.1, 1), typepoi='survey'), Poi(mediums=[-1], name='pp17', loc=(32.1, 143.2, -2), typepoi='sample'), Poi(mediums=[-1], name='pp18', loc=(28.8, 133.9, -2), typepoi='sample'), Poi(mediums=[-1, 1], name='sp21', loc=(26.5, 130.0, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp22', loc=(37.1, 137.5, 0), typepoi='transition')], robots=[], name='site6', center=(30.8, 138.1, 0), size=20), Site(poi=[Poi(mediums=[1], name='cpsite7', loc=(-275.3, -211.1, 1), typepoi='survey'), Poi(mediums=[-1], name='pp19', loc=(-275.7, -220.4, -2), typepoi='sample'), Poi(mediums=[-1], name='pp20', loc=(-267.1, -217.8, -2), typepoi='sample'), Poi(mediums=[-1, 1], name='sp23', loc=(-279.3, -214.0, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp24', loc=(-282.6, -208.3, 0), typepoi='transition')], robots=[], name='site7', center=(-275.3, -211.1, 0), size=20), Site(poi=[Poi(mediums=[1], name='cpsite8', loc=(-210.3, -244.5, 1), typepoi='survey'), Poi(mediums=[-1], name='pp21', loc=(-208.9, -245.1, -2), typepoi='sample'), Poi(mediums=[-1], name='pp22', loc=(-208.8, -246.6, -2), typepoi='sample'), Poi(mediums=[-1, 1], name='sp25', loc=(-220.1, -250.0, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp26', loc=(-218.8, -251.4, 0), typepoi='transition')], robots=[], name='site8', center=(-210.3, -244.5, 0), size=20), Site(poi=[Poi(mediums=[1], name='cpsite9', loc=(189.3, 407.6, 1), typepoi='survey'), Poi(mediums=[-1], name='pp23', loc=(189.2, 402.8, -1), typepoi='sample'), Poi(mediums=[-1, 1], name='sp27', loc=(193.1, 408.7, 0), typepoi='transition')], robots=[], name='site9', center=(189.3, 407.6, 0), size=10)], robots=[Robot(name='robot0', loc=(-469.2, -197.9, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot1', loc=(-469.2, -197.9, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot2', loc=(-469.2, -197.9, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot3', loc=(-469.2, -197.9, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot4', loc=(-469.2, -197.9, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot5', loc=(-469.2, -197.9, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot6', loc=(-469.2, -197.9, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot7', loc=(-469.2, -197.9, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot8', loc=(-469.2, -197.9, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot9', loc=(-469.2, -197.9, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000)], objective='assess', arenasize=1000, sitesize=(10, 50))

# print(getbestassignfrommission(mission=mission,minnbofcluster=2))