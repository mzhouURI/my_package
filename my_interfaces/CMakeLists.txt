cmake_minimum_required(VERSION 3.8)
project(my_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set (INTERFACES_LIST
  "srv/SimpleSrv.srv"
)

set (DEPENDENCIES_LIST
  "std_msgs"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${INTERFACES_LIST}
  DEPENDENCIES ${DEPENDENCIES_LIST}
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()
