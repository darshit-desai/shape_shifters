cmake_minimum_required(VERSION 3.8)
project(shape_shifters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ##############################################################################
# find dependencies
# ##############################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Doxygen REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

# ##############################################################################
# Set up level 1 and 2 unit test (i.e, integration test)
# ##############################################################################
# below sets up code coverage flags
if(COVERAGE)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "--coverage")
  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE "ON")
endif()

include_directories(include)

add_library(${PROJECT_NAME} SHARED libs/shapeshift.cpp src/leader.cpp
            src/robot.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  nav_msgs)

add_executable(main_node src/main.cpp src/robot.cpp src/leader.cpp
              libs/shapeshift.cpp)
ament_target_dependencies(
  main_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  nav_msgs)

target_link_libraries(main_node ${PROJECT_NAME} ${ament_LIBRARIES})
install(TARGETS main_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  ament_add_gtest(${PROJECT_NAME}_unit_test test/unit_test.cpp)
  target_include_directories(
    ${PROJECT_NAME}_unit_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_unit_test ${PROJECT_NAME}
                        ${ament_LIBRARIES})

  ament_add_gtest(${PROJECT_NAME}_integration_test test/integration_test.cpp)
  target_include_directories(
    ${PROJECT_NAME}_integration_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_integration_test ${PROJECT_NAME}
                        ${ament_LIBRARIES})
endif()

# ##############################################################################
# Create the compilation database for clangd and move it out of the build dir.
# ##############################################################################
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json
execute_process(
  COMMAND cmake -E create_symlink "${PROJECT_BINARY_DIR}/compile_commands.json"
          "${PROJECT_SOURCE_DIR}/compile_commands.json")

# ##############################################################################
# Doxygen Rules ref: https://cmake.org/cmake/help/latest/module/FindDoxygen.html
# ##############################################################################
if(DOXYGEN_FOUND)
  set(DOXYGEN_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/docs)
  set(DOXYGEN_COLLABORATION_GRAPH YES)
  set(DOXYGEN_EXTRACT_ALL YES)
  set(DOXYGEN_CLASS_DIAGRAMS YES)
  set(DOXYGEN_HIDE_UNDOC_RELATIONS NO)
  set(DOXYGEN_HAVE_DOT YES)
  set(DOXYGEN_CLASS_GRAPH YES)
  set(DOXYGEN_CALL_GRAPH YES)
  set(DOXYGEN_CALLER_GRAPH YES)
  set(DOXYGEN_COLLABORATION_GRAPH YES)
  set(DOXYGEN_BUILTIN_STL_SUPPORT YES)
  set(DOXYGEN_EXTRACT_PRIVATE YES)
  set(DOXYGEN_EXTRACT_PACKAGE YES)
  set(DOXYGEN_EXTRACT_STATIC YES)
  set(DOXYGEN_EXTRACT_LOCALMETHODS YES)
  set(DOXYGEN_UML_LOOK YES)
  set(DOXYGEN_UML_LIMIT_NUM_FIELDS 50)
  set(DOXYGEN_TEMPLATE_RELATIONS YES)
  set(DOXYGEN_DOT_GRAPH_MAX_NODES 100)
  set(DOXYGEN_MAX_DOT_GRAPH_DEPTH 0)
  set(DOXYGEN_DOT_TRANSPARENT YES)
else()
  message(
    FATAL_ERROR
      "Doxygen needs to be installed to generate the doxygen documentation")
endif()

# ##############################################################################
# Specify Targets
# ##############################################################################
# create a target to build documentation
doxygen_add_docs(
  docs # target name
  # List of files or directories
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/test)

# ##############################################################################
# Installation
# ##############################################################################

# install launch files
install(DIRECTORY launch worlds models DESTINATION share/${PROJECT_NAME})
# install scripts
install(PROGRAMS      # want to make the file executable (ie. chmod a+x)
  scripts/generate_coverage_report.bash
  DESTINATION lib/${PROJECT_NAME})
ament_package()
