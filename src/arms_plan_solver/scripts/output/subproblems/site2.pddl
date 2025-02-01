(define (problem TMMSproblemPREALLOC)
(:domain MMdomainextended)
(:objects

robot2 robot3 - robot 

site2 - site 

cpsite2 pp2 pp3 pp4 pp5 sp2 sp3 sp4 sp5 sp6 - pointofinterest 

)
(:init 

; Robot states 

(at robot2 cpsite2)
(at_site robot2 site2)
(airconf robot2)
(available robot2)

(at robot3 cpsite2)
(at_site robot3 site2)
(airconf robot3)
(available robot3)


; Poi/Sites states

;site2
(survey_poi cpsite2)
(air cpsite2)
(partofsite cpsite2 site2)

(sample_poi pp2)
(water pp2)
(partofsite pp2 site2)

(sample_poi pp3)
(water pp3)
(partofsite pp3 site2)

(sample_poi pp4)
(water pp4)
(partofsite pp4 site2)

(sample_poi pp5)
(water pp5)
(partofsite pp5 site2)

(transition_poi sp2)
(water sp2)
(air sp2)
(partofsite sp2 site2)

(transition_poi sp3)
(water sp3)
(air sp3)
(partofsite sp3 site2)

(transition_poi sp4)
(water sp4)
(air sp4)
(partofsite sp4 site2)

(transition_poi sp5)
(water sp5)
(air sp5)
(partofsite sp5 site2)

(transition_poi sp6)
(water sp6)
(air sp6)
(partofsite sp6 site2)


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

;site2

(= (site_size site2) 40)

(= (distance cpsite2 sp2) 23.22)
(= (distance cpsite2 sp3) 20.59)
(= (distance cpsite2 sp4) 13.19)
(= (distance cpsite2 sp5) 18.81)
(= (distance cpsite2 sp6) 11.12)

(= (distance pp2 pp3) 20.28)
(= (distance pp2 pp4) 6.24)
(= (distance pp2 pp5) 10.57)
(= (distance pp2 sp2) 44.36)
(= (distance pp2 sp3) 16.39)
(= (distance pp2 sp4) 32.36)
(= (distance pp2 sp5) 38.4)
(= (distance pp2 sp6) 11.06)

(= (distance pp3 pp2) 20.28)
(= (distance pp3 pp4) 14.07)
(= (distance pp3 pp5) 9.78)
(= (distance pp3 sp2) 26.96)
(= (distance pp3 sp3) 24.98)
(= (distance pp3 sp4) 19.76)
(= (distance pp3 sp5) 24.76)
(= (distance pp3 sp6) 10.96)

(= (distance pp4 pp2) 6.24)
(= (distance pp4 pp3) 14.07)
(= (distance pp4 pp5) 4.33)
(= (distance pp4 sp2) 38.95)
(= (distance pp4 sp3) 17.5)
(= (distance pp4 sp4) 27.87)
(= (distance pp4 sp5) 33.84)
(= (distance pp4 sp6) 6.01)

(= (distance pp5 pp2) 10.57)
(= (distance pp5 pp3) 9.78)
(= (distance pp5 pp4) 4.33)
(= (distance pp5 sp2) 35.36)
(= (distance pp5 sp3) 19.44)
(= (distance pp5 sp4) 25.2)
(= (distance pp5 sp5) 31.01)
(= (distance pp5 sp6) 4.68)

(= (distance sp2 cpsite2) 23.22)
(= (distance sp2 pp2) 44.36)
(= (distance sp2 pp3) 26.96)
(= (distance sp2 pp4) 38.95)
(= (distance sp2 pp5) 35.36)
(= (distance sp2 sp3) 39.4)
(= (distance sp2 sp4) 14.48)
(= (distance sp2 sp5) 10.48)
(= (distance sp2 sp6) 34.27)

(= (distance sp3 cpsite2) 20.59)
(= (distance sp3 pp2) 16.39)
(= (distance sp3 pp3) 24.98)
(= (distance sp3 pp4) 17.5)
(= (distance sp3 pp5) 19.44)
(= (distance sp3 sp2) 39.4)
(= (distance sp3 sp4) 25.09)
(= (distance sp3 sp5) 30.59)
(= (distance sp3 sp6) 16.77)

(= (distance sp4 cpsite2) 13.19)
(= (distance sp4 pp2) 32.36)
(= (distance sp4 pp3) 19.76)
(= (distance sp4 pp4) 27.87)
(= (distance sp4 pp5) 25.2)
(= (distance sp4 sp2) 14.48)
(= (distance sp4 sp3) 25.09)
(= (distance sp4 sp5) 6.08)
(= (distance sp4 sp6) 23.3)

(= (distance sp5 cpsite2) 18.81)
(= (distance sp5 pp2) 38.4)
(= (distance sp5 pp3) 24.76)
(= (distance sp5 pp4) 33.84)
(= (distance sp5 pp5) 31.01)
(= (distance sp5 sp2) 10.48)
(= (distance sp5 sp3) 30.59)
(= (distance sp5 sp4) 6.08)
(= (distance sp5 sp6) 29.27)

(= (distance sp6 cpsite2) 11.12)
(= (distance sp6 pp2) 11.06)
(= (distance sp6 pp3) 10.96)
(= (distance sp6 pp4) 6.01)
(= (distance sp6 pp5) 4.68)
(= (distance sp6 sp2) 34.27)
(= (distance sp6 sp3) 16.77)
(= (distance sp6 sp4) 23.3)
(= (distance sp6 sp5) 29.27)


)
(:goal (and
(assessed pp2)
(assessed pp3)
(assessed pp4)
(assessed pp5)
)
)
)
