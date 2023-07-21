cmake_minimum_required(VERSION 3.8)
project(my_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  my_interfaces
)

include_directories(include)
set(library_name ${PROJECT_NAME})

#Add library
add_library(${library_name} SHARED src/my_simple_node.cpp)
ament_target_dependencies(${library_name} ${dependencies})
#Add exe
add_executable(my_simple_node src/my_simple_node_main.cpp)
target_link_libraries(my_simple_node ${library_name})
ament_target_dependencies(my_simple_node ${dependencies})
# ##install target
install(TARGETS my_simple_node ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()