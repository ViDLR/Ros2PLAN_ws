cmake_minimum_required(VERSION 3.8)
project(user_visualization_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Find Graphviz package using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GRAPHVIZ REQUIRED libgvc)

include_directories(
  include
  ${Python3_INCLUDE_DIRS}
  ${GRAPHVIZ_INCLUDE_DIRS}
)

# Download matplotlibcpp.h if not available
set(MATPLOTLIBCPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
if(NOT EXISTS "${MATPLOTLIBCPP_DIR}/matplotlibcpp.h")
  message(STATUS "Downloading matplotlibcpp.h...")
  file(DOWNLOAD
    "https://raw.githubusercontent.com/lava/matplotlib-cpp/master/matplotlibcpp.h"
    "${MATPLOTLIBCPP_DIR}/matplotlibcpp.h"
  )
endif()

add_executable(uservisualization_node src/uservisualization_node.cpp)
ament_target_dependencies(uservisualization_node rclcpp std_msgs visualization_msgs plansys2_msgs)

target_link_libraries(uservisualization_node
  ${Python3_LIBRARIES}
  ${GRAPHVIZ_LIBRARIES}
)

install(TARGETS
  uservisualization_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

