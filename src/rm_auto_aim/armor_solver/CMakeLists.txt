cmake_minimum_required(VERSION 3.10)
project(armor_solver)

## Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -O3)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rm_utils REQUIRED)

find_path(EIGEN3_INCLUDE_DIR Eigen/Dense
  PATHS /usr/include/eigen3 /usr/local/include/eigen3 /opt/ros/humble/include/eigen3
)

if(NOT EIGEN3_INCLUDE_DIR)
  message(FATAL_ERROR "Eigen3 headers not found")
endif()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

add_library(tinympc STATIC
  third_party/tinympc/admm.cpp
  third_party/tinympc/tiny_api.cpp
  third_party/tinympc/codegen.cpp
  third_party/tinympc/rho_benchmark.cpp
)

set_property(TARGET tinympc PROPERTY POSITION_INDEPENDENT_CODE ON)
target_compile_options(tinympc PRIVATE -w)
target_include_directories(tinympc PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} tinympc)
ament_target_dependencies(${PROJECT_NAME} rm_utils)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fyt::auto_aim::ArmorSolverNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)

endif()

#############
## Install ##
#############

ament_auto_package()