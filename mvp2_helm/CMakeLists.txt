cmake_minimum_required(VERSION 3.5)
project(mvp2_helm)

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

set(library_name helm)
set(executable_name mvp2_helm)

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

include_directories(
  include
)

# Library
add_library(${library_name} SHARED
  include/mvp2_helm/helm.cpp
  include/mvp2_helm/node_wrapper.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

# Executable
add_executable(${executable_name}
  src/main.cpp
)
target_link_libraries(${executable_name} 
  ${library_name}
)
ament_target_dependencies(${executable_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "mvp2_mission::Helm")

# Install
install(TARGETS ${library_name} ${executable_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params 
  DESTINATION share/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   find_package(ament_cmake_gtest REQUIRED)
#   add_subdirectory(test)
# endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
