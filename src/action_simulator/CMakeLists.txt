cmake_minimum_required(VERSION 3.5)
project(action_simulator)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_core REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_popf_plan_solver REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(plansys2_executor REQUIRED)  # Added

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  plansys2_core
  plansys2_pddl_parser
  ament_index_cpp
  plansys2_msgs
  plansys2_domain_expert
  plansys2_problem_expert
  plansys2_popf_plan_solver
  pluginlib
  std_msgs
  lifecycle_msgs
  rosidl_default_runtime
  plansys2_executor  # Added
)

# Declare custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionExecutionInfo.msg"
)

ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Declare a C++ executable
add_executable(action_simulator_node src/action_simulator_node.cpp)
ament_target_dependencies(action_simulator_node  ${dependencies})

add_executable(manager_node src/manager_node.cpp)
ament_target_dependencies(manager_node ${dependencies})

# Link against plansys2_executor library
target_link_libraries(manager_node
  ${plansys2_executor_LIBRARIES}
)

# Get typesupport target for linking
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")

# Install executables and/or libraries
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS action_simulator_node
  manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

