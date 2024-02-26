add_library(${library_name} SHARED
  lifecycle_node.cpp
  node_thread.cpp
  node_utils.cpp
  robot_utils.cpp
)

ament_target_dependencies(${library_name}
  rclcpp
  tf2
  tf2_ros
  nav_msgs
  geometry_msgs
  lifecycle_msgs
  rclcpp_lifecycle
  tf2_geometry_msgs
  bondcpp
)

# add_executable(lifecycle_bringup
#   lifecycle_bringup_commandline.cpp
# )
# target_link_libraries(lifecycle_bringup ${library_name})

find_package(Boost REQUIRED COMPONENTS program_options)

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  # lifecycle_bringup
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
