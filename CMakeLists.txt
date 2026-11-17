cmake_minimum_required(VERSION 3.28...4.2)
project(uncertainty_planning_core VERSION 0.1.0 LANGUAGES CXX)

include(CMakePackageConfigHelpers)
include(CheckIPOSupported)
include(GNUInstallDirs)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(UNCERTAINTY_PLANNING_CORE_NATIVE_ARCH
       "Compile with -march=native (only safe if every consumer does too)" OFF)

find_package(common_robotics_utilities REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

# Defaults when this project is built on its own, as colcon does. A parent
# project or the command line can override them.
if(PROJECT_IS_TOP_LEVEL)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
  endif()
  if(NOT DEFINED CMAKE_COMPILE_WARNING_AS_ERROR)
    set(CMAKE_COMPILE_WARNING_AS_ERROR ON)
  endif()
  if(NOT DEFINED CMAKE_INTERPROCEDURAL_OPTIMIZATION)
    check_ipo_supported(RESULT ipo_supported)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ${ipo_supported})
  endif()
endif()

# Warnings for this project's own targets only; consumers don't inherit them.
add_library(upc_warnings INTERFACE)
set(upc_warning_flags
  -Wall -Wextra -Wconversion -Wshadow -Wold-style-cast -Wpessimizing-move
  -Wuninitialized -Werror=non-virtual-dtor)
target_compile_options(upc_warnings INTERFACE
  "$<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:${upc_warning_flags}>")

add_library(${PROJECT_NAME})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_sources(${PROJECT_NAME}
  PRIVATE
    src/${PROJECT_NAME}/uncertainty_planning_core.cpp
  PUBLIC
    FILE_SET HEADERS
    BASE_DIRS include
    FILES
      include/uncertainty_planning_core/execution_policy.hpp
      include/uncertainty_planning_core/ros_integration.hpp
      include/uncertainty_planning_core/simple_outcome_clustering_interface.hpp
      include/uncertainty_planning_core/simple_sampler_interface.hpp
      include/uncertainty_planning_core/simple_simulator_interface.hpp
      include/uncertainty_planning_core/task_planner_adapter.hpp
      include/uncertainty_planning_core/uncertainty_contact_planning.hpp
      include/uncertainty_planning_core/uncertainty_planner_state.hpp
      include/uncertainty_planning_core/uncertainty_planning_core.hpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    common_robotics_utilities::common_robotics_utilities
    Eigen3::Eigen
    rclcpp::rclcpp
    ${visualization_msgs_TARGETS}
  PRIVATE
    $<BUILD_INTERFACE:upc_warnings>)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(task_planner_adapter_example
  src/task_planner_adapter_example.cpp)
target_link_libraries(task_planner_adapter_example
  PRIVATE ${PROJECT_NAME} upc_warnings)

if(UNCERTAINTY_PLANNING_CORE_NATIVE_ARCH)
  target_compile_options(${PROJECT_NAME} PUBLIC -march=native)
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  FILE_SET HEADERS)
install(TARGETS task_planner_adapter_example)
install(EXPORT ${PROJECT_NAME}Targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake)
configure_package_config_file(
  cmake/${PROJECT_NAME}Config.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake)
write_basic_package_version_file(
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY SameMinorVersion)
install(FILES
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake)
install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
