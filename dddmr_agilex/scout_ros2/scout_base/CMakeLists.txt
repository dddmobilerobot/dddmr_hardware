cmake_minimum_required(VERSION 3.5)
project(scout_base)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/weston_robot/lib/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(scout_msgs REQUIRED)
find_package(ugv_sdk REQUIRED)

find_package(serial REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include_directories(
  include
)

set(DEPENDENCIES
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
  scout_msgs
  sensor_msgs
  serial
  rcl_interfaces
  tf2_geometry_msgs
)

# scout_messenger
add_library(scout_messenger SHARED
  src/scout_messenger.cpp
)
target_link_libraries(scout_messenger
  ${PCL_LIBRARIES}
  ugv_sdk
)
ament_target_dependencies(scout_messenger
  ${DEPENDENCIES}
)

add_executable(scout_base_node 
    src/scout_base_ros.cpp
    src/scout_base_node.cpp
)
target_link_libraries(scout_base_node  
  scout_messenger
)
ament_target_dependencies(scout_base_node 
  ${DEPENDENCIES}
)

install(TARGETS
  scout_messenger
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install node
install(TARGETS
  scout_base_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
