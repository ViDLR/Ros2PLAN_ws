(define (problem TMMSproblemPREALLOC)
(:domain MMdomainextended)
(:objects

robot2 robot3 - robot 

site3 - site 

cpsite3 pp6 pp7 pp8 pp9 sp7 sp8 sp9 sp10 sp11 - pointofinterest 

)
(:init 

; Robot states 

(at robot2 cpsite3)
(at_site robot2 site3)
(airconf robot2)
(available robot2)

(at robot3 cpsite3)
(at_site robot3 site3)
(airconf robot3)
(available robot3)


; Poi/Sites states

;site3
(survey_poi cpsite3)
(air cpsite3)
(partofsite cpsite3 site3)

(sample_poi pp6)
(water pp6)
(partofsite pp6 site3)

(sample_poi pp7)
(water pp7)
(partofsite pp7 site3)

(sample_poi pp8)
(water pp8)
(partofsite pp8 site3)

(sample_poi pp9)
(water pp9)
(partofsite pp9 site3)

(transition_poi sp7)
(water sp7)
(air sp7)
(partofsite sp7 site3)

(transition_poi sp8)
(water sp8)
(air sp8)
(partofsite sp8 site3)

(transition_poi sp9)
(water sp9)
(air sp9)
(partofsite sp9 site3)

(transition_poi sp10)
(water sp10)
(air sp10)
(partofsite sp10 site3)

(transition_poi sp11)
(water sp11)
(air sp11)
(partofsite sp11 site3)


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

;site3

(= (site_size site3) 40)

(= (distance cpsite3 sp7) 5.6)
(= (distance cpsite3 sp8) 16.22)
(= (distance cpsite3 sp9) 18.59)
(= (distance cpsite3 sp10) 15.18)
(= (distance cpsite3 sp11) 23.91)

(= (distance pp6 pp7) 26.1)
(= (distance pp6 pp8) 9.19)
(= (distance pp6 pp9) 30.06)
(= (distance pp6 sp7) 19.02)
(= (distance pp6 sp8) 9.86)
(= (distance pp6 sp9) 6.95)
(= (distance pp6 sp10) 30.68)
(= (distance pp6 sp11) 16.55)

(= (distance pp7 pp6) 26.1)
(= (distance pp7 pp8) 29.7)
(= (distance pp7 pp9) 15.81)
(= (distance pp7 sp7) 11.02)
(= (distance pp7 sp8) 25.32)
(= (distance pp7 sp9) 27.92)
(= (distance pp7 sp10) 9.2)
(= (distance pp7 sp11) 32.17)

(= (distance pp8 pp6) 9.19)
(= (distance pp8 pp7) 29.7)
(= (distance pp8 pp9) 37.21)
(= (distance pp8 sp7) 25.08)
(= (distance pp8 sp8) 18.63)
(= (distance pp8 sp9) 5.35)
(= (distance pp8 sp10) 31.97)
(= (distance pp8 sp11) 25.24)

(= (distance pp9 pp6) 30.06)
(= (distance pp9 pp7) 15.81)
(= (distance pp9 pp8) 37.21)
(= (distance pp9 sp7) 13.34)
(= (distance pp9 sp8) 24.37)
(= (distance pp9 sp9) 34.33)
(= (distance pp9 sp10) 24.03)
(= (distance pp9 sp11) 27.4)

(= (distance sp7 cpsite3) 5.6)
(= (distance sp7 pp6) 19.02)
(= (distance sp7 pp7) 11.02)
(= (distance sp7 pp8) 25.08)
(= (distance sp7 pp9) 13.34)
(= (distance sp7 sp8) 15.28)
(= (distance sp7 sp9) 21.85)
(= (distance sp7 sp10) 17.86)
(= (distance sp7 sp11) 21.7)

(= (distance sp8 cpsite3) 16.22)
(= (distance sp8 pp6) 9.86)
(= (distance sp8 pp7) 25.32)
(= (distance sp8 pp8) 18.63)
(= (distance sp8 pp9) 24.37)
(= (distance sp8 sp7) 15.28)
(= (distance sp8 sp9) 14.67)
(= (distance sp8 sp10) 31.33)
(= (distance sp8 sp11) 8.46)

(= (distance sp9 cpsite3) 18.59)
(= (distance sp9 pp6) 6.95)
(= (distance sp9 pp7) 27.92)
(= (distance sp9 pp8) 5.35)
(= (distance sp9 pp9) 34.33)
(= (distance sp9 sp7) 21.85)
(= (distance sp9 sp8) 14.67)
(= (distance sp9 sp10) 30.48)
(= (distance sp9 sp11) 21.6)

(= (distance sp10 cpsite3) 15.18)
(= (distance sp10 pp6) 30.68)
(= (distance sp10 pp7) 9.2)
(= (distance sp10 pp8) 31.97)
(= (distance sp10 pp9) 24.03)
(= (distance sp10 sp7) 17.86)
(= (distance sp10 sp8) 31.33)
(= (distance sp10 sp9) 30.48)
(= (distance sp10 sp11) 38.92)

(= (distance sp11 cpsite3) 23.91)
(= (distance sp11 pp6) 16.55)
(= (distance sp11 pp7) 32.17)
(= (distance sp11 pp8) 25.24)
(= (distance sp11 pp9) 27.4)
(= (distance sp11 sp7) 21.7)
(= (distance sp11 sp8) 8.46)
(= (distance sp11 sp9) 21.6)
(= (distance sp11 sp10) 38.92)


)
(:goal (and
(assessed pp6)
(assessed pp7)
(assessed pp8)
(assessed pp9)
)
)
)
