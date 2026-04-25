cmake_minimum_required(VERSION 3.8)
project(rm_omniperception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(armor_detector REQUIRED)
find_package(rm_utils REQUIRED)

add_library(omniperception_node SHARED
  src/omniperception_node.cpp
  src/omniperception/decider.cpp
  src/omniperception/perceptron.cpp
  src/usbcamera/usbcamera.cpp)
target_include_directories(omniperception_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(omniperception_node PUBLIC c_std_99 cxx_std_17)
target_link_libraries(omniperception_node ${OpenCV_LIBS})

ament_target_dependencies(
  omniperception_node
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "rm_interfaces"
  "armor_detector"
  "rm_utils"
)

rclcpp_components_register_node(
  omniperception_node
  PLUGIN "rm_omniperception::OmniPerceptionNode"
  EXECUTABLE rm_omniperception_node
)

install(TARGETS omniperception_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

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
