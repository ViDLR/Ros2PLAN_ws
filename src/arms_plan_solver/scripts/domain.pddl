(define (domain mmdomainextended)
(:requirements :disjunctive-preconditions :durative-actions :fluents :negative-preconditions :strips :typing :universal-preconditions )

(:types
robot
pointofinterest
site
)

(:predicates
(air ?poi - pointofinterest)
(airconf ?r - robot)
(at ?r - robot ?poi - pointofinterest)
(at_site ?r - robot ?s - site)
(available ?r - robot)
(canrelay ?r - robot)
(cansample ?r - robot)
(canswitch ?r - robot)
(connected ?s - site)
(ground ?poi - pointofinterest)
(groundconf ?r - robot)
(isrelay ?poi - pointofinterest)
(isswitchable ?poi - pointofinterest)
(observed ?s - site)
(partofsite ?poi - pointofinterest ?s - site)
(sample_poi ?poi - pointofinterest)
(sampled ?poi - pointofinterest)
(survey_poi ?poi - pointofinterest)
(transition_poi ?poi - pointofinterest)
(water ?poi - pointofinterest)
(waterconf ?r - robot)
)

(:functions
(distance ?poi1 ?poi2 - pointofinterest)
(site_size ?s - site)
(energy ?r - robot)
(recharge_rate ?r - robot)
(speedair ?r - robot)
(speedwater ?r - robot)
(maintainconsumption_air ?r - robot)
(maintainconsumption_water ?r - robot)
(assesscost ?r - robot )
(partassesscost ?r - robot)
(observecost ?r - robot)
(navconsumption_water ?r - robot)
(navconsumption_air ?r - robot)
(switchduration_airwater ?r - robot)
(switchcost_airwater ?r - robot)
(landingduration_airground ?r - robot)
(landingcost_airground ?r - robot)
(takeoffduration_groundair ?r - robot)
(takeoffost_groundair ?r - robot)
(switchduration_waterair ?r - robot)
(switchcost_waterair ?r - robot)
)

(:durative-action navigation_air
	:parameters (?r - robot ?poi1 ?poi2 - pointofinterest ?s - site)
	:duration ( = ?duration (/ (distance ?poi1 ?poi2) (speedair ?r)))
	:condition (and
		(at start(available ?r))
		(at start (partofsite ?poi1 ?s))
		(at start (partofsite ?poi2 ?s))
		(at start (at ?r ?poi1))
		(at start (air ?poi1))
		(at start (air ?poi2))
		(at start (airconf ?r))
		
	)
	:effect (and
		
        (at start (not (available ?r)))
        (at start (not (at ?r ?poi1)))
        (at end (at ?r ?poi2))
        (at end (available ?r))
    )
)
(:durative-action navigation_water
	:parameters (?r - robot ?poi1 ?poi2 - pointofinterest ?s - site)
	:duration ( = ?duration (/ (distance ?poi1 ?poi2) (speedwater ?r)))
	:condition (and
		(at start(available ?r))
		(at start (partofsite ?poi1 ?s))
		(at start (partofsite ?poi2 ?s))
		(at start (at ?r ?poi1))
		(at start (water ?poi1))
		(at start (water ?poi2))
		(at start (waterconf ?r))
		
	)
	:effect (and
		
        (at start (not (available ?r)))
        (at start (not (at ?r ?poi1)))
        (at end (at ?r ?poi2))
        (at end (available ?r))
    )
)
(:durative-action takeoff
	:parameters (?r - robot ?poi - pointofinterest)
	:duration ( = ?duration (takeoffduration_groundair ?r))
	:condition (and
		(at start (available ?r))
		(at start (at ?r ?poi))
		(at start (transition_poi ?poi))
		(at start (groundconf ?r))
		(at start (air ?poi))
		(at start (ground ?poi))
		
	)
	:effect (and
		
        (at start (not (available ?r)))
        (at start (not (groundconf ?r)))
        (at end (airconf ?r))
		(at end (at ?r ?poi))
        (at end (available ?r))
		(at end (available ?r))
    )
)
(:durative-action landing
	:parameters (?r - robot ?poi - pointofinterest)
	:duration ( = ?duration (landingduration_airground ?r))
	:condition (and
		(at start(available ?r))
		(over all (at ?r ?poi))
		(over all (transition_poi ?poi))
		(at start (airconf ?r))
		(over all (air ?poi))
		(over all (ground ?poi))
		
	)
	:effect (and
		
        (at start (not (available ?r)))
        (at start (not (airconf ?r)))
        (at end (groundconf ?r))
        (at end (available ?r))
    )
)
(:durative-action observe
	:parameters (?r - robot ?poi - pointofinterest ?s - site)
	:duration ( = ?duration (/ (site_size ?s) (speedair ?r)))
	:condition (and
		(at start (at ?r ?poi))
		(at start (at_site ?r ?s))
		(at start (survey_poi ?poi))
		(at start (partofsite ?poi ?s))
		(at start (air ?poi))
		(at start (available ?r))
		(at start (airconf ?r))
		
	)
	:effect (and
		(at start (not(available ?r)))
		
		(at end (available ?r))
		(at end (observed ?s))
	)
)
(:durative-action observe_2r
	:parameters (?r1 ?r2 - robot ?poi - pointofinterest ?s - site)
	:duration (= ?duration (/ (/ (site_size ?s) (speedair ?r1)) 2))
	:condition (and
		(at start (at ?r1 ?poi))
		(at start (at_site ?r1 ?s))
		(at start (at ?r2 ?poi))
		(at start (at_site ?r2 ?s))
		(at start (survey_poi ?poi))
		(at start (partofsite ?poi ?s))
		(at start (air ?poi))
		(at start (available ?r1))
		(at start (available ?r2))
		(at start (airconf ?r1))
		(at start (airconf ?r2))
		
		
	)
	:effect (and
		(at start (not(available ?r1)))
		(at start (not(available ?r2)))
		
		
		(at end (available ?r1))
		(at end (available ?r2))
		(at end (observed ?s))
	)
)
(:durative-action switch_waterair
	:parameters (?r - robot ?poi - pointofinterest)
	:duration ( = ?duration (switchduration_waterair ?r))
	:condition (and
		(at start (available ?r))
		(over all (canswitch ?r))
		(at start (waterconf ?r))
		(over all (at ?r ?poi))
		(over all (transition_poi ?poi))
		(over all (isswitchable ?poi))
		(over all (water ?poi))
		(over all (air ?poi))
		
	)
	:effect (and
		
        (at start (not (available ?r)))
        (at start (not (waterconf ?r)))
        (at end (airconf ?r))
        (at end (available ?r))
    )
)
(:durative-action switch_airwater
	:parameters (?r - robot ?poi - pointofinterest ?s - site)
	:duration ( = ?duration (switchduration_airwater ?r))
	:condition (and
		(at start(available ?r))
		(over all (canswitch ?r))
		(at start (airconf ?r))
		(over all (at ?r ?poi))
		(over all (observed ?s))
		(over all (transition_poi ?poi))
		(over all (isswitchable ?poi))
		(over all (air ?poi))
		(over all (water ?poi))
		
	)
	:effect (and
		
        (at start (not (available ?r)))
        (at start (not (airconf ?r)))
        (at end (waterconf ?r))
        (at end (available ?r))
    )
)
(:durative-action translate_data
	:parameters (?r - robot ?poi - pointofinterest ?s - site)
	:duration ( = ?duration 45)
	:condition (and
		(over all (at ?r ?poi))
		(over all (transition_poi ?poi))
		(over all (partofsite ?poi ?s))
		(over all (observed ?s))
		(over all (isrelay ?poi))
		(at start (available ?r))
		(over all (waterconf ?r))
		(over all (canrelay ?r))
		
	)
	:effect (and
		(at start (not (available ?r)))
		(at start (connected ?s))
		
		(at end (not (connected ?s)))
		(at end (available ?r))
	)
)
(:durative-action sample
	:parameters (?r - robot ?poi - pointofinterest ?s - site)
	:duration ( = ?duration 30)
	:condition (and
		(over all (at ?r ?poi))
		(at start (available ?r))
		(over all (cansample ?r))
		(over all (waterconf ?r))
		(over all (water ?poi))
		(over all (sample_poi ?poi))
		(over all (partofsite ?poi ?s))
		(at start (connected ?s))
		
	)
	:effect (and
		(at start (not(available ?r)))
		
		(at end (available ?r))
		(at end (sampled ?poi))
	)
)
(:durative-action change_site
	:parameters (?r - robot ?s1 ?s2 - site ?poi1 ?poi2 - pointofinterest)
	:duration ( = ?duration (/ (distance ?poi1 ?poi2) (speedair ?r)))
	:condition (and
		(at start (at_site ?r ?s1))
		(at start (at ?r ?poi1))
		(over all (air ?poi1))
		(over all (air ?poi2))
		(over all (partofsite ?poi1 ?s1))
		(over all (partofsite ?poi2 ?s2))
		(at start (available ?r))
		(over all (airconf ?r))
		
	)
	:effect (and
		(at start (not(available ?r)))
		(at start (not(at_site ?r ?s1)))
		(at start (not(at ?r ?poi1)))
		
		(at end (available ?r))
		(at end (at_site ?r ?s2))
		(at end (at ?r ?poi2))
	)
)
)
