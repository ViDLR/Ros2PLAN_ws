cmake_minimum_required(VERSION 3.5)

project(arms_plan_solver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_core REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/arms_plan_solver.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  plansys2_msgs
  pluginlib
  lifecycle_msgs
  ament_index_cpp
  plansys2_core)

include_directories(
  include
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(plansys2_core arms_plan_solver_plugin.xml)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY arms_utils/
  DESTINATION share/${PROJECT_NAME}/arms_utils
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY external_solver/
  DESTINATION lib/${PROJECT_NAME}/external_solver
  USE_SOURCE_PERMISSIONS
)

install(FILES arms_plan_solver_plugin.xml
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
