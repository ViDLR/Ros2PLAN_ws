(define (problem test_problem)
  (:domain test_domain)
  (:objects
    robot1 robot2 - robot
    entrance kitchen bedroom dinning bathroom chargingroom - room
  )
  (:init
    (connected entrance dinning)
    (connected dinning entrance)
    (connected dinning kitchen)
    (connected kitchen dinning)
    (connected dinning bedroom)
    (connected bedroom dinning)
    (connected bathroom bedroom)
    (connected bedroom bathroom)
    (connected chargingroom kitchen)
    (connected kitchen chargingroom)
    (charging_point_at chargingroom)
    (battery_low robot1)
    (battery_full robot2)
    (robot_at robot1 entrance)
    (robot_at robot2 kitchen)
  )
  (:goal
    (and
      (robot_at robot1 bathroom)
      (robot_at robot2 entrance)
    )
  )
)

