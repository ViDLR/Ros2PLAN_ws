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
(beaconed ?s - site)

;; robot predicates

(at ?r - robot ?poi - pointofinterest)
(at_site ?r - robot ?s - site)
(airconf ?r - robot)
(groundconf ?r - robot)
(available ?r - robot)
(canobs ?r - robot)
(hasbeacons ?r - robot)
(camera_ready ?r - robot)

;; Poi predicates

(transition_poi ?poi - pointofinterest)
(survey_poi ?poi - pointofinterest)
(air ?poi - pointofinterest)
(checked ?poi - pointofinterest)
(ground ?poi - pointofinterest)
(partofsite ?poi - pointofinterest ?s - site)


);; end Predicates ;;;;;;;;;;;;;;;;;;;;

;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions
(distance ?poi1 ?poi2 - pointofinterest)

(site_size ?s - site)
(speedair ?r - robot)
(landingduration_airground ?r - robot)
(takeoffduration_groundair ?r - robot)

);; end Functions ;;;;;;;;;;;;;;;;;;;;

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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


(:durative-action check_transition_point
	:parameters (?r - robot ?poi - pointofinterest ?s - site)
	:duration ( = ?duration 30)
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

(:durative-action Pose_beacon
	:parameters (?r1 ?r2 - robot ?poi - pointofinterest ?s - site)
	:duration ( = ?duration 30)
	:condition (and
		(over all (at ?r1 ?poi))
		(at start (available ?r1))
		(over all (at ?r2 ?poi))
		(at start (available ?r2))
		(at start (hasbeacons ?r1))
		(over all (airconf ?r1))
		(over all (airconf ?r2))
		(over all (air ?poi))
		(over all (transition_poi ?poi))
		(over all (partofsite ?poi ?s))
		(at start (observed ?s))
		(at start (checked ?poi))
		(over all (camera_ready ?r2))
	)
	:effect (and
		(at start (not(available ?r1)))
		(at start (not(available ?r2)))
		(at end (available ?r1))
		(at end (available ?r2))
		(at end (beaconed ?s))
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


);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
