(define (problem TMMSproblemPREALLOC)
(:domain MMdomainextended)
(:objects

robot0 robot1 - robot 

site4 - site 

cpsite4 pp10 pp11 pp12 pp13 sp12 sp13 sp14 sp15 sp16 - pointofinterest 

)
(:init 

; Robot states 

(at robot0 cpsite4)
(at_site robot0 site4)
(airconf robot0)
(available robot0)

(at robot1 cpsite4)
(at_site robot1 site4)
(airconf robot1)
(available robot1)


; Poi/Sites states

;site4
(survey_poi cpsite4)
(air cpsite4)
(partofsite cpsite4 site4)

(sample_poi pp10)
(water pp10)
(partofsite pp10 site4)

(sample_poi pp11)
(water pp11)
(partofsite pp11 site4)

(sample_poi pp12)
(water pp12)
(partofsite pp12 site4)

(sample_poi pp13)
(water pp13)
(partofsite pp13 site4)

(transition_poi sp12)
(water sp12)
(air sp12)
(partofsite sp12 site4)

(transition_poi sp13)
(water sp13)
(air sp13)
(partofsite sp13 site4)

(transition_poi sp14)
(water sp14)
(air sp14)
(partofsite sp14 site4)

(transition_poi sp15)
(water sp15)
(air sp15)
(partofsite sp15 site4)

(transition_poi sp16)
(water sp16)
(air sp16)
(partofsite sp16 site4)


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

;site4

(= (site_size site4) 40)

(= (distance cpsite4 sp12) 19.05)
(= (distance cpsite4 sp13) 22.13)
(= (distance cpsite4 sp14) 11.41)
(= (distance cpsite4 sp15) 18.72)
(= (distance cpsite4 sp16) 18.31)

(= (distance pp10 pp11) 14.37)
(= (distance pp10 pp12) 20.69)
(= (distance pp10 pp13) 24.24)
(= (distance pp10 sp12) 22.14)
(= (distance pp10 sp13) 34.63)
(= (distance pp10 sp14) 20.62)
(= (distance pp10 sp15) 20.5)
(= (distance pp10 sp16) 10.13)

(= (distance pp11 pp10) 14.37)
(= (distance pp11 pp12) 14.02)
(= (distance pp11 pp13) 13.3)
(= (distance pp11 sp12) 21.02)
(= (distance pp11 sp13) 21.94)
(= (distance pp11 sp14) 10.56)
(= (distance pp11 sp15) 20.56)
(= (distance pp11 sp16) 18.86)

(= (distance pp12 pp10) 20.69)
(= (distance pp12 pp11) 14.02)
(= (distance pp12 pp13) 26.0)
(= (distance pp12 sp12) 9.56)
(= (distance pp12 sp13) 31.46)
(= (distance pp12 sp14) 23.55)
(= (distance pp12 sp15) 10.26)
(= (distance pp12 sp16) 17.99)

(= (distance pp13 pp10) 24.24)
(= (distance pp13 pp11) 13.3)
(= (distance pp13 pp12) 26.0)
(= (distance pp13 sp12) 33.83)
(= (distance pp13 sp13) 11.44)
(= (distance pp13 sp14) 5.68)
(= (distance pp13 sp15) 33.55)
(= (distance pp13 sp16) 31.08)

(= (distance sp12 cpsite4) 19.05)
(= (distance sp12 pp10) 22.14)
(= (distance sp12 pp11) 21.02)
(= (distance sp12 pp12) 9.56)
(= (distance sp12 pp13) 33.83)
(= (distance sp12 sp13) 39.67)
(= (distance sp12 sp14) 30.37)
(= (distance sp12 sp15) 2.06)
(= (distance sp12 sp16) 15.0)

(= (distance sp13 cpsite4) 22.13)
(= (distance sp13 pp10) 34.63)
(= (distance sp13 pp11) 21.94)
(= (distance sp13 pp12) 31.46)
(= (distance sp13 pp13) 11.44)
(= (distance sp13 sp12) 39.67)
(= (distance sp13 sp14) 14.41)
(= (distance sp13 sp15) 39.88)
(= (distance sp13 sp16) 39.97)

(= (distance sp14 cpsite4) 11.41)
(= (distance sp14 pp10) 20.62)
(= (distance sp14 pp11) 10.56)
(= (distance sp14 pp12) 23.55)
(= (distance sp14 pp13) 5.68)
(= (distance sp14 sp12) 30.37)
(= (distance sp14 sp13) 14.41)
(= (distance sp14 sp15) 29.94)
(= (distance sp14 sp16) 26.83)

(= (distance sp15 cpsite4) 18.72)
(= (distance sp15 pp10) 20.5)
(= (distance sp15 pp11) 20.56)
(= (distance sp15 pp12) 10.26)
(= (distance sp15 pp13) 33.55)
(= (distance sp15 sp12) 2.06)
(= (distance sp15 sp13) 39.88)
(= (distance sp15 sp14) 29.94)
(= (distance sp15 sp16) 13.01)

(= (distance sp16 cpsite4) 18.31)
(= (distance sp16 pp10) 10.13)
(= (distance sp16 pp11) 18.86)
(= (distance sp16 pp12) 17.99)
(= (distance sp16 pp13) 31.08)
(= (distance sp16 sp12) 15.0)
(= (distance sp16 sp13) 39.97)
(= (distance sp16 sp14) 26.83)
(= (distance sp16 sp15) 13.01)


)
(:goal (and
(assessed pp10)
(assessed pp11)
(assessed pp12)
(assessed pp13)
)
)
)
