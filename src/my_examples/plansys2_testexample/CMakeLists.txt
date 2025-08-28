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
    plansys2_executor
    plansys2_lifecycle_manager
)

# Add include directories
include_directories(include)

set(ACTION_NODE_SOURCES
  src/change_site_action_node.cpp
  src/navigation_air_action_node.cpp
  src/navigation_water_action_node.cpp
  src/landing_action_node.cpp
  src/takeoff_action_node.cpp
  src/switch_airwater_action_node.cpp
  src/switch_waterair_action_node.cpp
  src/translate_data_action_node.cpp
  src/observe_action_node.cpp
  src/observe_2r_action_node.cpp
  src/sample_action_node.cpp
)

# Group all action node sources into a single library
set(ACTION_NODE_EXECUTABLES
  src/change_site_action_node_exec.cpp
  src/navigation_air_action_node_exec.cpp
  src/navigation_water_action_node_exec.cpp
  src/landing_action_node_exec.cpp
  src/takeoff_action_node_exec.cpp
  src/switch_airwater_action_node_exec.cpp
  src/switch_waterair_action_node_exec.cpp
  src/translate_data_action_node_exec.cpp
  src/observe_action_node_exec.cpp
  src/observe_2r_action_node_exec.cpp
  src/sample_action_node_exec.cpp
)

# === Shared action node logic library
add_library(action_nodes ${ACTION_NODE_SOURCES})
ament_target_dependencies(action_nodes ${dependencies})

# === Build and install each standalone action node executable
foreach(exec_src ${ACTION_NODE_EXECUTABLES})
  get_filename_component(exec_name ${exec_src} NAME_WE)
  add_executable(${exec_name} ${exec_src})
  ament_target_dependencies(${exec_name} ${dependencies})
  target_link_libraries(${exec_name} action_nodes)
  install(TARGETS ${exec_name}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endforeach()


add_executable(plansys2_core_bringup src/plansys2_core_bringup.cpp)
ament_target_dependencies(plansys2_core_bringup ${dependencies})

add_executable(dem_bringup src/dem_bringup.cpp)
ament_target_dependencies(dem_bringup ${dependencies})

add_executable(team_lifecycle_manager_node src/team_lifecycle_manager_node.cpp)
ament_target_dependencies(team_lifecycle_manager_node ${dependencies})
target_compile_definitions(team_lifecycle_manager_node PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(team_lifecycle_manager_node action_nodes)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)


install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

# === Install main system targets
install(TARGETS
  action_nodes
  team_lifecycle_manager_node
  plansys2_core_bringup
  dem_bringup
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

