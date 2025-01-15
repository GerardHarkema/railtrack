cmake_minimum_required(VERSION 3.8)
project(railway_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackProtocolDefines.msg"
  "msg/LocomotiveControl.msg"
  "msg/LocomotiveState.msg"
  "msg/TurnoutControl.msg"
  "msg/TurnoutState.msg"
  "msg/PowerControl.msg"
  "msg/PowerState.msg"
  "msg/DccCvWrite.msg"
  "msg/TrackObjectConfig.msg"
  "msg/TrackConfig.msg"
  "msg/ColorRGB.msg"
  "msg/SceneryControl.msg"
  "msg/SceneryState.msg"
   DEPENDENCIES std_msgs
 )


ament_package()
