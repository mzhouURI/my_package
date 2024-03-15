cmake_minimum_required(VERSION 3.5)
project(bhv_template)

find_package(ament_cmake REQUIRED)
find_package(mvp2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mvp2_util REQUIRED)
find_package(mvp_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behavior_interface REQUIRED)
find_package(pluginlib REQUIRED)

mvp2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  mvp2_util
  mvp_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  behavior_interface
  pluginlib
)

# plugins
add_library(mvp2_bhv_template SHARED
  src/bhv_template.cpp
)

ament_target_dependencies(mvp2_bhv_template
  ${dependencies}
)


pluginlib_export_plugin_description_file(behavior_interface behavior_plugin.xml)

install(TARGETS 
  mvp2_bhv_template
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY src/
  DESTINATION share/${PROJECT_NAME}/src/
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   find_package(ament_cmake_gtest REQUIRED)
#   add_subdirectory(test)
# endif()

ament_export_include_directories(include)
ament_export_libraries(
  mvp2_bhv_template
)
ament_export_dependencies(${dependencies})
ament_package()
