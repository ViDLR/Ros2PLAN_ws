(define (problem TMMSproblemPREALLOC)
(:domain MMdomainextended)
(:objects

robot0 robot1 - robot 

site6 - site 

cpsite6 pp15 pp16 sp18 sp19 - pointofinterest 

)
(:init 

; Robot states 

(at robot0 cpsite6)
(at_site robot0 site6)
(airconf robot0)
(available robot0)

(at robot1 cpsite6)
(at_site robot1 site6)
(airconf robot1)
(available robot1)


; Poi/Sites states

;site6
(survey_poi cpsite6)
(air cpsite6)
(partofsite cpsite6 site6)

(sample_poi pp15)
(water pp15)
(partofsite pp15 site6)

(sample_poi pp16)
(water pp16)
(partofsite pp16 site6)

(transition_poi sp18)
(water sp18)
(air sp18)
(partofsite sp18 site6)

(transition_poi sp19)
(water sp19)
(air sp19)
(partofsite sp19 site6)


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

;DistancesPoiSites

;site6

(= (site_size site6) 20)

(= (distance cpsite6 sp18) 12.38)
(= (distance cpsite6 sp19) 10.07)

(= (distance pp15 pp16) 8.55)
(= (distance pp15 sp18) 18.48)
(= (distance pp15 sp19) 18.57)

(= (distance pp16 pp15) 8.55)
(= (distance pp16 sp18) 21.87)
(= (distance pp16 sp19) 17.58)

(= (distance sp18 cpsite6) 12.38)
(= (distance sp18 pp15) 18.48)
(= (distance sp18 pp16) 21.87)
(= (distance sp18 sp19) 10.11)

(= (distance sp19 cpsite6) 10.07)
(= (distance sp19 pp15) 18.57)
(= (distance sp19 pp16) 17.58)
(= (distance sp19 sp18) 10.11)


)
(:goal (and
(assessed pp15)
(assessed pp16)
)
)
)
