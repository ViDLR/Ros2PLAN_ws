(define (problem TMMSproblemPREALLOC)
(:domain MMdomainextended)
(:objects

robot2 robot3 - robot 

site5 - site 

cpsite5 pp14 sp17 - pointofinterest 

)
(:init 

; Robot states 

(at robot2 cpsite5)
(at_site robot2 site5)
(airconf robot2)
(available robot2)

(at robot3 cpsite5)
(at_site robot3 site5)
(airconf robot3)
(available robot3)


; Poi/Sites states

;site5
(survey_poi cpsite5)
(air cpsite5)
(partofsite cpsite5 site5)

(sample_poi pp14)
(water pp14)
(partofsite pp14 site5)

(transition_poi sp17)
(water sp17)
(air sp17)
(partofsite sp17 site5)


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

;site5

(= (site_size site5) 10)

(= (distance cpsite5 sp17) 1.79)

(= (distance pp14 sp17) 2.52)

(= (distance sp17 cpsite5) 1.79)
(= (distance sp17 pp14) 2.52)


)
(:goal (and
(assessed pp14)
)
)
)
