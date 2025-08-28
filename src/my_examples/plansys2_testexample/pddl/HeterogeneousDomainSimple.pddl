(define (domain MMdomainextended)
(:requirements :strips :typing :fluents :negative-preconditions :disjunctive-preconditions :durative-actions :universal-preconditions )

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types

robot
pointofinterest
site

);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

;; site predicates

(observed ?s - site)

;; robot predicates

(at ?r - robot ?poi - pointofinterest)
(at_site ?r - robot ?s - site)
(waterconf ?r - robot)
(airconf ?r - robot)
(groundconf ?r - robot)
(available ?r - robot)
(canrelay ?r - robot)
(cansample ?r - robot)
(canobs ?r - robot)

;; Poi predicates

(sample_poi ?poi - pointofinterest)
(transition_poi ?poi - pointofinterest)
(survey_poi ?poi - pointofinterest)
(water ?poi - pointofinterest)
(air ?poi - pointofinterest)
(ground ?poi - pointofinterest)
(checked ?poi - pointofinterest)
(connected ?s - site)
(partofsite ?poi - pointofinterest ?s - site)
(sampled ?poi - pointofinterest)
(isrelay ?poi - pointofinterest)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;

;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions
(distance ?poi1 ?poi2 - pointofinterest)

(site_size ?s - site)

(speedair ?r - robot)
(speedwater ?r - robot)
(landingduration_airground ?r - robot)
(takeoffduration_groundair ?r - robot)

);; end Functions ;;;;;;;;;;;;;;;;;;;;

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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

(:durative-action translate_data
	:parameters (?r - robot ?poi - pointofinterest ?s - site)
	:duration ( = ?duration 45)
	:condition (and
		(over all (at ?r ?poi))
		(over all (transition_poi ?poi))
		(over all (partofsite ?poi ?s))
		(over all (observed ?s))
		(over all  (checked ?poi))
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
		(over all (connected ?s))
	)
	:effect (and
		(at start (not(available ?r)))
		(at end (available ?r))
		(at end (sampled ?poi))
	)
)

(:durative-action check_transition_point
	:parameters (?r - robot ?poi - pointofinterest ?s - site)
	:duration ( = ?duration 8)
	:condition (and
		(over all (at ?r ?poi))
		(at start (available ?r))
		(over all (canobs ?r))
		(over all (airconf ?r))
		(over all (air ?poi))
		(over all (transition_poi ?poi))
		(over all (partofsite ?poi ?s))
		(over all (observed ?s))
	)
	:effect (and
		(at start (not(available ?r)))
		(at end (available ?r))
		(at end (checked ?poi))
	)
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
