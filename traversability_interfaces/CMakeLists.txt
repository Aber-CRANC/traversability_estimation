cmake_minimum_required(VERSION 3.5)
project(traversability_interfaces)

find_package(ament_cmake_auto)
ament_auto_find_build_dependencies()

set(srv_files
  "srv/CheckFootprintPath.srv"
  "srv/Overwrite.srv"
)

set(msg_files
  "msg/FootprintPath.msg"
  "msg/TraversabilityResult.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
