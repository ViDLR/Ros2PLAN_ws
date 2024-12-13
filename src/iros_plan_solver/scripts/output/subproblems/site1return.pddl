(define (problem TMMSproblemPREALLOC)
(:domain MMdomainextended)
(:objects

robot2 robot3 - robot 

site1 - site 

cpsite1 pp1 sp1 - pointofinterest 

)
(:init 

; Robot states 

(at robot2 pp1)
(at_site robot2 site1)
(waterconf robot2)
(available robot2)

(at robot3 sp1)
(at_site robot3 site1)
(waterconf robot3)
(available robot3)


; Poi/Sites states

;site1
(survey_poi cpsite1)
(air cpsite1)
(partofsite cpsite1 site1)

(sample_poi pp1)
(water pp1)
(partofsite pp1 site1)

(transition_poi sp1)
(water sp1)
(air sp1)
(partofsite sp1 site1)


;Functions

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

;site1

(= (site_size site1) 10)

(= (distance cpsite1 sp1) 5.29)

(= (distance pp1 sp1) 7.08)

(= (distance sp1 cpsite1) 5.29)
(= (distance sp1 pp1) 7.08)


)
(:goal (and
(at robot2 cpsite1)
(airconf robot2)
(at robot3 cpsite1)
(airconf robot3)
)
)
)