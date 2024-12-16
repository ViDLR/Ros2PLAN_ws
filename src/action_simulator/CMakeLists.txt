cmake_minimum_required(VERSION 3.5)

project(action_simulator)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_lifecycle_manager REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

set(dependencies
  rclcpp
  plansys2_msgs
  lifecycle_msgs
  plansys2_domain_expert
  plansys2_problem_expert
  plansys2_planner
  plansys2_executor
  plansys2_lifecycle_manager
)

# Include directories
include_directories(include)

# Source files
set(SOURCES
  src/action_simulator/ExecutionManagerNode.cpp
)

# Create shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Create executable nodes
add_executable(execution_manager_node src/execution_manager_node.cpp)
ament_target_dependencies(execution_manager_node ${dependencies})
target_compile_definitions(execution_manager_node PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(execution_manager_node ${PROJECT_NAME})

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install library and executables
install(TARGETS
  ${PROJECT_NAME}
  execution_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Export information
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()

