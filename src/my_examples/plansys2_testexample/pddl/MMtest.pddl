; mission = Mission(sites=[Site(poi=[Poi(mediums=[0, 1], name='cpbase', loc=(410.8, 212.9, 0), typepoi='transition')], robots=[], name='base', center=(410.8, 212.9, 0), size=10), Site(poi=[Poi(mediums=[1], name='cpsite1', loc=(471.7, 277.9, 1), typepoi='survey'), Poi(mediums=[-1], name='pp1', loc=(465.8, 270.3, -2), typepoi='sample'), Poi(mediums=[-1], name='pp2', loc=(476.7, 285.3, -2), typepoi='sample'), Poi(mediums=[-1, 1], name='sp1', loc=(466.7, 281.0, 0), typepoi='transition'), Poi(mediums=[-1, 1], name='sp2', loc=(476.5, 281.0, 0), typepoi='transition')], robots=[], name='site1', center=(471.7, 277.9, 0), size=20)], robots=[Robot(name='robot0', loc=(410.8, 212.9, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot1', loc=(410.8, 212.9, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot2', loc=(410.8, 212.9, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000), Robot(name='robot3', loc=(410.8, 212.9, 0), medium=0, histo=[], currentpathcost=0, poi='cpbase', site='base', energy=10000)], objective='assess', arenasize=1000, sitesize=(10, 50))
(define (problem TMMSproblem)
(:domain MMdomainextended)
(:objects

robot0 robot1 robot2 robot3 - robot 

base site1 - site 

cpbase cpsite1 pp1 pp2 sp1 sp2 - pointofinterest 

)
(:init 

; Robot states 

(at robot0 cpbase)
(at_site robot0 base)
(groundconf robot0)
(available robot0)

(at robot1 cpbase)
(at_site robot1 base)
(groundconf robot1)
(available robot1)

(at robot2 cpbase)
(at_site robot2 base)
(groundconf robot2)
(available robot2)

(at robot3 cpbase)
(at_site robot3 base)
(groundconf robot3)
(available robot3)


; Poi/Sites states

;base
(transition_poi cpbase)
(ground cpbase)
(air cpbase)
(partofsite cpbase base)


;site1
(survey_poi cpsite1)
(air cpsite1)
(partofsite cpsite1 site1)

(sample_poi pp1)
(water pp1)
(partofsite pp1 site1)

(sample_poi pp2)
(water pp2)
(partofsite pp2 site1)

(transition_poi sp1)
(water sp1)
(air sp1)
(partofsite sp1 site1)

(transition_poi sp2)
(water sp2)
(air sp2)
(partofsite sp2 site1)


;Functions

;robot0
;Speeds
(= (speedair robot0) 1.5)
(= (speedwater robot0) 0.5)
;Energy
(= (energy robot0) 1000000000)
(= (recharge_rate robot0) 10)
;Cost
(= (assesscost robot0) 0.02)
(= (partassesscost robot0) 0.01)
(= (observecost robot0) 0.01)
;Consumption
(= (maintainconsumption_water robot0) 0.02)
(= (maintainconsumption_air robot0) 0.08)
(= (navconsumption_water robot0) 0.5)
(= (navconsumption_air robot0) 0.8)
;Switch
(= (switchduration_airwater robot0) 5)
(= (switchcost_airwater robot0) 20)
(= (takeoffduration_groundair robot0) 4)
(= (takeoffost_groundair robot0) 10)
(= (landingduration_airground robot0) 3)
(= (landingcost_airground robot0) 5)
(= (switchduration_waterair robot0) 8)
(= (switchcost_waterair robot0) 30)

;robot1
;Speeds
(= (speedair robot1) 1.5)
(= (speedwater robot1) 0.5)
;Energy
(= (energy robot1) 1000000000)
(= (recharge_rate robot1) 10)
;Cost
(= (assesscost robot1) 0.02)
(= (partassesscost robot1) 0.01)
(= (observecost robot1) 0.01)
;Consumption
(= (maintainconsumption_water robot1) 0.02)
(= (maintainconsumption_air robot1) 0.08)
(= (navconsumption_water robot1) 0.5)
(= (navconsumption_air robot1) 0.8)
;Switch
(= (switchduration_airwater robot1) 5)
(= (switchcost_airwater robot1) 20)
(= (takeoffduration_groundair robot1) 4)
(= (takeoffost_groundair robot1) 10)
(= (landingduration_airground robot1) 3)
(= (landingcost_airground robot1) 5)
(= (switchduration_waterair robot1) 8)
(= (switchcost_waterair robot1) 30)

;robot2
;Speeds
(= (speedair robot2) 1.5)
(= (speedwater robot2) 0.5)
;Energy
(= (energy robot2) 1000000000)
(= (recharge_rate robot2) 10)
;Cost
(= (assesscost robot2) 0.02)
(= (partassesscost robot2) 0.01)
(= (observecost robot2) 0.01)
;Consumption
(= (maintainconsumption_water robot2) 0.02)
(= (maintainconsumption_air robot2) 0.08)
(= (navconsumption_water robot2) 0.5)
(= (navconsumption_air robot2) 0.8)
;Switch
(= (switchduration_airwater robot2) 5)
(= (switchcost_airwater robot2) 20)
(= (takeoffduration_groundair robot2) 4)
(= (takeoffost_groundair robot2) 10)
(= (landingduration_airground robot2) 3)
(= (landingcost_airground robot2) 5)
(= (switchduration_waterair robot2) 8)
(= (switchcost_waterair robot2) 30)

;robot3
;Speeds
(= (speedair robot3) 1.5)
(= (speedwater robot3) 0.5)
;Energy
(= (energy robot3) 1000000000)
(= (recharge_rate robot3) 10)
;Cost
(= (assesscost robot3) 0.02)
(= (partassesscost robot3) 0.01)
(= (observecost robot3) 0.01)
;Consumption
(= (maintainconsumption_water robot3) 0.02)
(= (maintainconsumption_air robot3) 0.08)
(= (navconsumption_water robot3) 0.5)
(= (navconsumption_air robot3) 0.8)
;Switch
(= (switchduration_airwater robot3) 5)
(= (switchcost_airwater robot3) 20)
(= (takeoffduration_groundair robot3) 4)
(= (takeoffost_groundair robot3) 10)
(= (landingduration_airground robot3) 3)
(= (landingcost_airground robot3) 5)
(= (switchduration_waterair robot3) 8)
(= (switchcost_waterair robot3) 30)

;DistancesPoiSites

;base

(= (site_size base) 10)

(= (distance cpbase cpsite1) 89.08)
(= (distance cpbase sp1) 88.1)
(= (distance cpbase sp2) 94.63)

;site1

(= (site_size site1) 20)

(= (distance cpsite1 cpbase) 89.08)
(= (distance cpsite1 sp1) 5.97)
(= (distance cpsite1 sp2) 5.8)

(= (distance pp1 pp2) 18.54)
(= (distance pp1 sp1) 10.92)
(= (distance pp1 sp2) 15.26)

(= (distance pp2 pp1) 18.54)
(= (distance pp2 sp1) 11.07)
(= (distance pp2 sp2) 4.75)

(= (distance sp1 cpbase) 88.1)
(= (distance sp1 cpsite1) 5.97)
(= (distance sp1 pp1) 10.92)
(= (distance sp1 pp2) 11.07)
(= (distance sp1 sp2) 9.8)

(= (distance sp2 cpbase) 94.63)
(= (distance sp2 cpsite1) 5.8)
(= (distance sp2 pp1) 15.26)
(= (distance sp2 pp2) 4.75)
(= (distance sp2 sp1) 9.8)


)
(:goal (and
(assessed pp1)
(assessed pp2)
)
)
)
