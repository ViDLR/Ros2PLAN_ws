cmake_minimum_required(VERSION 3.5)
project(plansys2_testexample)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(action_simulator REQUIRED)
find_package(plansys2_executor REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    plansys2_msgs
    plansys2_executor
    action_simulator
)

add_executable(navigation_water_action_node src/navigation_water_action_node.cpp)
ament_target_dependencies(navigation_water_action_node ${dependencies})

add_executable(navigation_air_action_node src/navigation_air_action_node.cpp)
ament_target_dependencies(navigation_air_action_node ${dependencies})

add_executable(landing_action_node src/landing_action_node.cpp)
ament_target_dependencies(landing_action_node ${dependencies})

add_executable(takeoff_action_node src/takeoff_action_node.cpp)
ament_target_dependencies(takeoff_action_node ${dependencies})

add_executable(switch_airwater_action_node src/switch_airwater_action_node.cpp)
ament_target_dependencies(switch_airwater_action_node ${dependencies})

add_executable(switch_waterair_action_node src/switch_waterair_action_node.cpp)
ament_target_dependencies(switch_waterair_action_node ${dependencies})

add_executable(translate_data_action_node src/translate_data_action_node.cpp)
ament_target_dependencies(translate_data_action_node ${dependencies})

add_executable(observe_action_node src/observe_action_node.cpp)
ament_target_dependencies(observe_action_node ${dependencies})

add_executable(observe_2r_action_node src/observe_2r_action_node.cpp)
ament_target_dependencies(observe_2r_action_node ${dependencies})

add_executable(sample_action_node src/sample_action_node.cpp)
ament_target_dependencies(sample_action_node ${dependencies})

add_executable(change_site_action_node src/change_site_action_node.cpp)
ament_target_dependencies(change_site_action_node ${dependencies})

add_executable(plansys2_core_bringup src/plansys2_core_bringup.cpp)
ament_target_dependencies(plansys2_core_bringup ${dependencies})

add_executable(emn_bringup src/emn_bringup.cpp)
ament_target_dependencies(emn_bringup ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  navigation_water_action_node
  navigation_air_action_node
  landing_action_node
  takeoff_action_node
  switch_waterair_action_node
  switch_airwater_action_node
  translate_data_action_node
  observe_action_node
  observe_2r_action_node
  sample_action_node
  change_site_action_node
  plansys2_core_bringup
  emn_bringup
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()

